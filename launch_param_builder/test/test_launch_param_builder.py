# Copyright 2023 PickNik
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the PickNik nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


from launch_param_builder import ParameterBuilder


def test_builder():
    parameters = (
        ParameterBuilder("launch_param_builder")
        .parameter("my_parameter", 20.0)
        .file_parameter(
            "parameter_file", "config/parameter_file"
        )  # Or /absolute/path/to/file
        .yaml(file_path="config/parameters.yaml")  # Or /absolute/path/to/file
        .xacro_parameter(
            parameter_name="my_robot",
            file_path="config/parameter.xacro",  # Or /absolute/path/to/file
            mappings={"prefix": "robot"},
        )
        .to_dict()
    )

    assert parameters["my_parameter"] == 20.0, "Parameter not loaded"
    assert parameters.get("parameter_file") is not None, "Parameter file not loaded"
    assert len(parameters["ros2_versions"]) == 7, "Parameter yaml file not loaded"
    assert parameters["the_answer_to_life"] == 42
    assert parameters["package_name"] == "launch_param_builder"
    assert parameters.get("my_robot") is not None, "Parameter xacro not loaded"
