FROM ros:lyrical

# setup steelhead
RUN mkdir -p /subbots
COPY . /subbots/steelhead
WORKDIR /subbots/steelhead/docker
RUN chmod +x build.sh
RUN /bin/bash build.sh

# setup entrypoint
WORKDIR /subbots/steelhead
RUN chmod +x docker/entrypoint.sh

# Use entrypoint (which sources local install) to run cmd 
ENTRYPOINT ["/subbots/steelhead/docker/entrypoint.sh"]
CMD ["/bin/bash"]
