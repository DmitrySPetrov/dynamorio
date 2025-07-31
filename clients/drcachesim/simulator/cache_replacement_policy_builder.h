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

// Implementation of Builder pattern for cache replacement policy objects construction

#ifndef _CACHE_REPLACEMENT_POLICY_BUILDER_H_
#define _CACHE_REPLACEMENT_POLICY_BUILDER_H_

#include "cache_replacement_policy.h"
#include <memory>
#include <string>
#include <typeinfo>

namespace dynamorio {
namespace drmemtrace {

// Base class for construction of cache replacement policy object
class cache_replacement_policy_builder_base_t
{
public:
    cache_replacement_policy_builder_base_t() = default;
    virtual ~cache_replacement_policy_builder_base_t() = default;

    // Construct the object with specified cache sets and associativity
    virtual std::unique_ptr<cache_replacement_policy_t>
    construct(int num_sets, int associativity)=0;

    // Parse specified `value` from string to the type defined for the parameter `name`
    // and later use it for creation of new cache replacement policy objects.
    // If specified parameter name is not defined for cache replacement policy,
    //   returns false and set `*type` to nullptr.
    // If specified parameter name is defined for policy, but the value cannot be parsed,
    //   returns false and set `*type` to pointer to the type_info of the expected type.
    // Otherwise returns true
    virtual bool
    configure(const std::string& name, const std::string& value, const std::type_info** type)=0;
};

} // namespace drmemtrace
} // namespace dynamorio

#endif
