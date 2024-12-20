#include "HashTable.cuh"

namespace Algorithm
{
    namespace HashTable
    {
        uint64_t encode_key(const Eigen::Vector3f& position, float voxelSize)
        {
            int64_t x_voxel = static_cast<int64_t>(std::round(position.x() * 10.0f));
            int64_t y_voxel = static_cast<int64_t>(std::round(position.y() * 10.0f));
            int64_t z_voxel = static_cast<int64_t>(std::round(position.z() * 10.0f));

            uint64_t key = 0;
            key |= ((x_voxel  & 0x1FFFFF) << 42); // 21 bits for x
            key |= ((y_voxel  & 0x1FFFFF) << 21); // 21 bits for y
            key |= (z_voxel  & 0x1FFFFF);         // 21 bits for z
            return key;
        }

        Eigen::Vector3f decode_key(uint64_t key, float voxelSize)
        {
            int64_t x = (key >> 42) & 0x1FFFFF;
            int64_t y = (key >> 21) & 0x1FFFFF;
            int64_t z = key & 0x1FFFFF;

            // Adjust for signed values if necessary
            if (x & (1 << 20)) x -= (1 << 21);
            if (y & (1 << 20)) y -= (1 << 21);
            if (z & (1 << 20)) z -= (1 << 21);

            return { x * voxelSize, y * voxelSize, z * voxelSize };
        }

        // 32 bit Murmur3 hash
        __device__ uint32_t hash_32(uint32_t k)
        {
            k ^= k >> 16;
            k *= 0x85ebca6b;
            k ^= k >> 13;
            k *= 0xc2b2ae35;
            k ^= k >> 16;
            return k & (kHashTableCapacity - 1);
        }

        // 64 bit Murmur3 hash
        __device__ uint64_t hash_64(uint64_t key)
        {
            key ^= key >> 33;
            key *= 0xff51afd7ed558ccdULL;
            key ^= key >> 33;
            key *= 0xc4ceb9fe1a85ec53ULL;
            key ^= key >> 33;
            return key & (kHashTableCapacity - 1);
        }

        // Create a hash table. For linear probing, this is just an array of KeyValues
        KeyValue* create_hashtable()
        {
            // Allocate memory
            KeyValue* hashtable;
            cudaMalloc(&hashtable, sizeof(KeyValue) * kHashTableCapacity);

            // Initialize hash table to empty
            static_assert(kEmpty == 0xffffffff, "memset expected kEmpty=0xffffffff");
            cudaMemset(hashtable, 0xff, sizeof(KeyValue) * kHashTableCapacity);

            return hashtable;
        }

        // Insert the key/values in kvs into the hashtable
        __global__ void gpu_hashtable_insert(KeyValue* hashtable, const KeyValue* kvs, unsigned int numkvs)
        {
            unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
            if (threadid < numkvs)
            {
                uint64_t key = kvs[threadid].key;
                uint64_t value = kvs[threadid].value;
                uint64_t slot = hash_64(key);

                while (true)
                {
                    uint64_t prev = atomicCAS(&hashtable[slot].key, kEmpty, key);
                    if (prev == kEmpty || prev == key)
                    {
                        hashtable[slot].value = value;
                        return;
                    }

                    slot = (slot + 1) & (kHashTableCapacity - 1);
                }
            }
        }

        void insert_hashtable(KeyValue* pHashTable, const KeyValue* kvs, uint64_t num_kvs)
        {
            // Copy the keyvalues to the GPU
            KeyValue* device_kvs;
            cudaMalloc(&device_kvs, sizeof(KeyValue) * num_kvs);
            cudaMemcpy(device_kvs, kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyHostToDevice);

            // Have CUDA calculate the thread block size
            int mingridsize;
            int threadblocksize;
            cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, gpu_hashtable_insert, 0, 0);

            // Create events for GPU timing
            cudaEvent_t start, stop;
            cudaEventCreate(&start);
            cudaEventCreate(&stop);

            cudaEventRecord(start);

            // Insert all the keys into the hash table
            int gridsize = ((uint64_t)num_kvs + threadblocksize - 1) / threadblocksize;
            gpu_hashtable_insert << <gridsize, threadblocksize >> > (pHashTable, device_kvs, (uint64_t)num_kvs);

            cudaEventRecord(stop);

            cudaEventSynchronize(stop);

            float milliseconds = 0;
            cudaEventElapsedTime(&milliseconds, start, stop);
            float seconds = milliseconds / 1000.0f;
            printf("    GPU inserted %d items in %f ms (%f million keys/second)\n",
                num_kvs, milliseconds, num_kvs / (double)seconds / 1000000.0f);

            cudaFree(device_kvs);
        }

        // Lookup keys in the hashtable, and return the values
        __global__ void gpu_hashtable_lookup(KeyValue* hashtable, KeyValue* kvs, unsigned int numkvs)
        {
            unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
            if (threadid < numkvs)
            {
                uint64_t key = kvs[threadid].key;
                uint64_t slot = hash_64(key);

                while (true)
                {
                    if (hashtable[slot].key == key)
                    {
                        kvs[threadid].value = hashtable[slot].value;
                        return;
                    }
                    if (hashtable[slot].key == kEmpty)
                    {
                        kvs[threadid].value = kEmpty;
                        return;
                    }
                    slot = (slot + 1) & (kHashTableCapacity - 1);
                }
            }
        }

        void lookup_hashtable(KeyValue* pHashTable, KeyValue* kvs, uint64_t num_kvs)
        {
            // Copy the keyvalues to the GPU
            KeyValue* device_kvs;
            cudaMalloc(&device_kvs, sizeof(KeyValue) * num_kvs);
            cudaMemcpy(device_kvs, kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyHostToDevice);

            // Have CUDA calculate the thread block size
            int mingridsize;
            int threadblocksize;
            cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, gpu_hashtable_insert, 0, 0);

            // Create events for GPU timing
            cudaEvent_t start, stop;
            cudaEventCreate(&start);
            cudaEventCreate(&stop);

            cudaEventRecord(start);

            // Insert all the keys into the hash table
            int gridsize = ((uint64_t)num_kvs + threadblocksize - 1) / threadblocksize;
            gpu_hashtable_lookup << <gridsize, threadblocksize >> > (pHashTable, device_kvs, (uint64_t)num_kvs);

            cudaEventRecord(stop);

            cudaEventSynchronize(stop);

            float milliseconds = 0;
            cudaEventElapsedTime(&milliseconds, start, stop);
            float seconds = milliseconds / 1000.0f;
            printf("    GPU lookup %d items in %f ms (%f million keys/second)\n",
                num_kvs, milliseconds, num_kvs / (double)seconds / 1000000.0f);

            cudaFree(device_kvs);
        }

        // Delete each key in kvs from the hash table, if the key exists
        // A deleted key is left in the hash table, but its value is set to kEmpty
        // Deleted keys are not reused; once a key is assigned a slot, it never moves
        __global__ void gpu_hashtable_delete(KeyValue* hashtable, const KeyValue* kvs, unsigned int numkvs)
        {
            unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
            if (threadid < numkvs)
            {
                uint64_t key = kvs[threadid].key;
                uint64_t slot = hash_64(key);

                while (true)
                {
                    if (hashtable[slot].key == key)
                    {
                        hashtable[slot].value = kEmpty;
                        return;
                    }
                    if (hashtable[slot].key == kEmpty)
                    {
                        return;
                    }
                    slot = (slot + 1) & (kHashTableCapacity - 1);
                }
            }
        }

        void delete_hashtable(KeyValue* pHashTable, const KeyValue* kvs, uint64_t num_kvs)
        {
            // Copy the keyvalues to the GPU
            KeyValue* device_kvs;
            cudaMalloc(&device_kvs, sizeof(KeyValue) * num_kvs);
            cudaMemcpy(device_kvs, kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyHostToDevice);

            // Have CUDA calculate the thread block size
            int mingridsize;
            int threadblocksize;
            cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, gpu_hashtable_insert, 0, 0);

            // Create events for GPU timing
            cudaEvent_t start, stop;
            cudaEventCreate(&start);
            cudaEventCreate(&stop);

            cudaEventRecord(start);

            // Insert all the keys into the hash table
            int gridsize = ((uint64_t)num_kvs + threadblocksize - 1) / threadblocksize;
            gpu_hashtable_delete << <gridsize, threadblocksize >> > (pHashTable, device_kvs, (uint64_t)num_kvs);

            cudaEventRecord(stop);

            cudaEventSynchronize(stop);

            float milliseconds = 0;
            cudaEventElapsedTime(&milliseconds, start, stop);
            float seconds = milliseconds / 1000.0f;
            printf("    GPU delete %d items in %f ms (%f million keys/second)\n",
                num_kvs, milliseconds, num_kvs / (double)seconds / 1000000.0f);

            cudaFree(device_kvs);
        }

        // Iterate over every item in the hashtable; return non-empty key/values
        __global__ void gpu_iterate_hashtable(KeyValue* pHashTable, KeyValue* kvs, uint64_t* kvs_size)
        {
            unsigned int threadid = blockIdx.x * blockDim.x + threadIdx.x;
            if (threadid < kHashTableCapacity)
            {
                if (pHashTable[threadid].key != kEmpty)
                {
                    uint64_t value = pHashTable[threadid].value;
                    if (value != kEmpty)
                    {
                        uint64_t size = atomicAdd(kvs_size, 1);
                        kvs[size] = pHashTable[threadid];
                    }
                }
            }
        }

        std::vector<KeyValue> iterate_hashtable(KeyValue* pHashTable)
        {
            uint64_t* device_num_kvs;
            cudaMalloc(&device_num_kvs, sizeof(uint64_t));
            cudaMemset(device_num_kvs, 0, sizeof(uint64_t));

            KeyValue* device_kvs;
            cudaMalloc(&device_kvs, sizeof(KeyValue) * kNumKeyValues);

            int mingridsize;
            int threadblocksize;
            cudaOccupancyMaxPotentialBlockSize(&mingridsize, &threadblocksize, gpu_iterate_hashtable, 0, 0);

            int gridsize = (kHashTableCapacity + threadblocksize - 1) / threadblocksize;
            gpu_iterate_hashtable << <gridsize, threadblocksize >> > (pHashTable, device_kvs, device_num_kvs);

            uint64_t num_kvs;
            cudaMemcpy(&num_kvs, device_num_kvs, sizeof(uint64_t), cudaMemcpyDeviceToHost);

            std::vector<KeyValue> kvs;
            kvs.resize(num_kvs);

            cudaMemcpy(kvs.data(), device_kvs, sizeof(KeyValue) * num_kvs, cudaMemcpyDeviceToHost);

            cudaFree(device_kvs);
            cudaFree(device_num_kvs);

            return kvs;
        }

        // Free the memory of the hashtable
        void destroy_hashtable(KeyValue* pHashTable)
        {
            cudaFree(pHashTable);
        }
    }
}
