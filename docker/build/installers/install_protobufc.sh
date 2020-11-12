#!/usr/bin/env bash

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

# Fail on first error.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")"

wget -O protobuf-c-v1.3.3.tar.gz https://github.com/protobuf-c/protobuf-c/archive/v1.3.3.tar.gz
tar xzf protobuf-c-v1.3.3.tar.gz
pushd protobuf-c-1.3.3
./configure
make -j8
make install
popd

# Clean up.
rm -fr protobuf-c-v1.3.3.tar.gz protobuf-c-v1.3.3
