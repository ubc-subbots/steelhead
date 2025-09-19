FROM ros:foxy

# setup steelhead
RUN mkdir -p /subbots
COPY build.sh /subbots
WORKDIR /subbots
RUN chmod +x build.sh
RUN /bin/bash build.sh

# setup entrypoint
COPY entrypoint.sh /subbots
WORKDIR /subbots
RUN chmod +x entrypoint.sh

# Use entrypoint (which sources local install) to run cmd 
ENTRYPOINT ["/subbots/entrypoint.sh"]
CMD ["/bin/bash"]