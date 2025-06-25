#!/usr/bin/env python

# Copyright 2025 The DLRG team. All rights reserved.
# Licensed under the MIT License.
# You may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://opensource.org/licenses/MIT
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Enable `magiclaw.__version__` to get the version of the package."""

from importlib.metadata import PackageNotFoundError, version

try:
    __version__ = version("magiclaw")
except PackageNotFoundError:
    __version__ = "unknown"
