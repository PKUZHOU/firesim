AGFI Metadata/Tagging
================================================

In the AWS EC2 case, when you build an AGFI in FireSim, the AGFI description stored by AWS is
populated with metadata that helps the manager decide how to deploy
a simulation. The important metadata is listed below, along with how each field
is set and used:

- ``firesim-buildquadruplet``: This always reflects the quadruplet combination used to BUILD the AGFI.
- ``firesim-deployquadruplet``: This reflects the quadruplet combination that is used to DEPLOY the AGFI. By default, this is the same as ``firesim-buildquadruplet``. In certain cases however, your users may not have access to a particular configuration, but a simpler configuration may be sufficient for building a compatible software driver (e.g. if you have proprietary RTL in your FPGA image that doesn't interface with the outside system). In this case, you can specify a custom deployquadruplet at build time. If you do not do so, the manager will automatically set this to be the same as ``firesim-buildquadruplet``.
- ``firesim-commit``: This is the commit hash of the version of FireSim used to build this AGFI. If the AGFI was created from a dirty copy of the FireSim repo, "-dirty" will be appended to the commit hash.

