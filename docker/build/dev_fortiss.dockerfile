FROM apolloauto/apollo:dev-18.04-x86_64-20200602_0644

RUN curl https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - \
  && curl https://bazel.build/bazel-release.pub.gpg |apt-key add - \
  &&  apt-get update -y && apt-get install -y \
     tmux \
     htop \
     can-utils

# WORKDIR /apollo # otherwise max depth exceeded
