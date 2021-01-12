# BARK integration

bark will be integrated as a planning service inside the apollo planner. This documentation is a work-in-progress document and not a final documentation of a running version!

## Integration Strategy
We compile the necessary functions from bark into a library and link this library in apollo. We do not build bark alongside with apollo (old boost version, old gcc, old bazel, dependency hazzle, ...). I made the integration like boost is integrated into apollo.

Bark is located at /apollo/bark;
the shared library under /lib
and the headers under /include

In the WORKSPACE.in file, there is a new_local_repository "bark" that should point you to all the necessary locations.

## Workflow

1. In bark, there is a bazel rule building the shared library libcorec.so, located in the bark/BUILD file. Build it.
2. In the folder bazel-bin/bark copy the library in the apollo docker container: ```docker cp libcorec.so  apollo_dev_kessler:/apollo/bark/lib```
3. In apollo, add the dependency ```@bark//:core``` in the BUILD file and an include in the source file
4. Build apollo

## Important

* Append the library path in the docker container: ```export LD_LIBRARY_PATH=/apollo/bark/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}```

