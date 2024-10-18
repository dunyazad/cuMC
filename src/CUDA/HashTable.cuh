#pragma once

#include <stdio.h>
#include <fstream>
#include <iostream>
#include <vector>

#include <cuda_runtime.h>
#include <nvtx3/nvToolsExt.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/IterativeLinearSolvers>

namespace Algorithm
{
    namespace HashTable
    {
        struct KeyValue
        {
            uint64_t key;
            uint64_t value;
        };

        const uint64_t kHashTableCapacity = 128 * 1024 * 1024;

        const uint64_t kNumKeyValues = kHashTableCapacity / 2;

        const uint64_t kEmpty = 0xffffffff;

        KeyValue* create_hashtable();

        void insert_hashtable(KeyValue* hashtable, const KeyValue* kvs, uint64_t num_kvs);

        void lookup_hashtable(KeyValue* hashtable, KeyValue* kvs, uint64_t num_kvs);

        void delete_hashtable(KeyValue* hashtable, const KeyValue* kvs, uint64_t num_kvs);

        std::vector<KeyValue> iterate_hashtable(KeyValue* hashtable);

        void destroy_hashtable(KeyValue* hashtable);
    }
}
