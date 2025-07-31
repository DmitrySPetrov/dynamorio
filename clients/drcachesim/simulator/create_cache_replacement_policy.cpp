/* **********************************************************
 * Copyright (c) 2015-2025 Google, Inc.  All rights reserved.
 * **********************************************************/

/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Google, Inc. nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL VMWARE, INC. OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 */

#include "create_cache_replacement_policy.h"
#include "cache_replacement_policy_builder.h"

#include <memory>
#include <string>

#include "cache_replacement_policy.h"
#include "policy_bit_plru.h"
#include "policy_fifo.h"
#include "policy_lfu.h"
#include "policy_lru.h"
#include "options.h"
#include "parse_value.h"

namespace dynamorio {
namespace drmemtrace {

// Template class for construction of cache replacement policy objects.
// Default implementation is without extra parameters
template<typename T>
class cache_replacement_policy_builder_t: public cache_replacement_policy_builder_base_t
{
    std::unique_ptr<cache_replacement_policy_t>
    construct(int num_sets, int associativity) override
    {
        return std::unique_ptr<T>(new T(num_sets, associativity));
    }

    bool
    configure(const std::string& name, const std::string& value, const std::type_info** type) override
    {
        // No extra parameters defined
        return false;
    }
};

template<>
class cache_replacement_policy_builder_t<policy_bit_plru_t>:
    public cache_replacement_policy_builder_base_t
{
public:
    std::unique_ptr<cache_replacement_policy_t>
    construct(int num_sets, int associativity) override
    {
        return std::unique_ptr<policy_bit_plru_t>
            (new policy_bit_plru_t(num_sets, associativity, seed_));
    }

    bool
    configure(const std::string& name, const std::string& value, const std::type_info** type) override
    {
        if (name == "seed") {
            if (type) {
                *type = &typeid(int);
            }
            return parse_value<int>(value, &seed_);
        } else {
            *type = nullptr;
            return false;
        }
    }

private:
    int seed_;
};

std::unique_ptr<cache_replacement_policy_builder_base_t>
create_cache_replacement_policy_builder(const std::string &policy)
{
    // default LRU
    if (policy.empty() || policy == REPLACE_POLICY_LRU) {
        using builder = cache_replacement_policy_builder_t<policy_lru_t>;
        return std::unique_ptr<cache_replacement_policy_builder_base_t>(new builder{});
    }
    if (policy == REPLACE_POLICY_LFU) {
        using builder = cache_replacement_policy_builder_t<policy_lfu_t>;
        return std::unique_ptr<cache_replacement_policy_builder_base_t>(new builder{});
    }
    if (policy == REPLACE_POLICY_FIFO) {
        using builder = cache_replacement_policy_builder_t<policy_fifo_t>;
        return std::unique_ptr<cache_replacement_policy_builder_base_t>(new builder{});
    }
    if (policy == REPLACE_POLICY_BIT_PLRU) {
        using builder = cache_replacement_policy_builder_t<policy_bit_plru_t>;
        return std::unique_ptr<cache_replacement_policy_builder_base_t>(new builder{});
    }
    return nullptr;
}

std::unique_ptr<cache_replacement_policy_t>
create_cache_replacement_policy(const std::string &policy, int num_sets,
                                int associativity)
{
    auto builder = create_cache_replacement_policy_builder(policy);
    if (builder) {
        return builder->construct(num_sets, associativity);
    }
    return nullptr;
}

} // namespace drmemtrace
} // namespace dynamorio
