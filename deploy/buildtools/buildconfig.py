from time import strftime, gmtime
import pprint
from importlib import import_module

from awstools.awstools import *
from buildtools.buildfarmhostdispatcher import BuildFarmHostDispatcher

# imports needed for python type checking
from typing import Set, Type, Any, Optional, Dict, TYPE_CHECKING
# needed to avoid type-hint circular dependencies
# TODO: Solved in 3.7.+ by "from __future__ import annotations" (see https://stackoverflow.com/questions/33837918/type-hints-solve-circular-dependency)
#       and normal "import <module> as ..." syntax (see https://www.reddit.com/r/Python/comments/cug90e/how_to_not_create_circular_dependencies_when/)
if TYPE_CHECKING:
    from buildtools.buildconfigfile import BuildConfigFile
else:
    BuildConfigFile = object

def inheritors(klass: Type[Any]) -> Set[Type[Any]]:
    """Determine the subclasses that inherit from the input class.
    This is taken from https://stackoverflow.com/questions/5881873/python-find-all-classes-which-inherit-from-this-one.

    Args:
        klass: Input class.

    Returns:
        Set of subclasses that inherit from input class.
    """
    subclasses = set()
    work = [klass]
    while work:
        parent = work.pop()
        for child in parent.__subclasses__():
            if child not in subclasses:
                subclasses.add(child)
                work.append(child)
    return subclasses

class BuildConfig:
    """Represents a single build configuration used to build RTL, drivers, and bitstreams.

    Attributes:
        name: Name of config i.e. name of `config_build_recipe.yaml` section.
        build_config_file: Pointer to global build config file.
        TARGET_PROJECT: Target project to build.
        DESIGN: Design to build.
        TARGET_CONFIG: Target config to build.
        deploytriplet: Deploy triplet override.
        launch_time: Launch time of the manager.
        PLATFORM_CONFIG: Platform config to build.
        s3_bucketname: S3 bucketname for AFI builds.
        post_build_hook: Post build hook script.
        build_farm_name: Name of build farm given by user.
        build_farm_host_dispatcher: Build farm host dispatcher object.
    """
    name: str
    build_config_file: BuildConfigFile
    TARGET_PROJECT: Optional[str]
    DESIGN: str
    TARGET_CONFIG: str
    deploytriplet: Optional[str]
    launch_time: str
    PLATFORM_CONFIG: str
    s3_bucketname: str
    post_build_hook: str
    build_farm_name: str
    build_farm_host_dispatcher: BuildFarmHostDispatcher

    def __init__(self,
            name: str,
            recipe_config_dict: Dict[str, Any],
            build_farm_name: str,
            build_farm_config_file: Dict[str, Any],
            build_config_file: BuildConfigFile,
            launch_time: str) -> None:
        """
        Args:
            name: Name of config i.e. name of `config_build_recipe.yaml` section.
            recipe_config_dict: `config_build_recipe.yaml` options associated with name.
            build_farm_name: Name of build farm. Corresponds to top-level key of specific build farm in `config_build_farm.yaml` file.
            build_farm_config_file: Parsed representation of `config_build_farm.yaml` file.
            build_config_file: Global build config file.
            launch_time: Time manager was launched.
        """
        self.name = name
        self.build_config_file = build_config_file

        self.TARGET_PROJECT = recipe_config_dict.get('TARGET_PROJECT')
        self.DESIGN = recipe_config_dict['DESIGN']
        self.TARGET_CONFIG = recipe_config_dict['TARGET_CONFIG']
        self.deploytriplet = recipe_config_dict['deploy-triplet']
        self.launch_time = launch_time

        # run platform specific options
        self.PLATFORM_CONFIG = recipe_config_dict['PLATFORM_CONFIG']
        self.s3_bucketname = recipe_config_dict['s3-bucket-name']
        if valid_aws_configure_creds():
            aws_resource_names_dict = aws_resource_names()
            if aws_resource_names_dict['s3bucketname'] is not None:
                # in tutorial mode, special s3 bucket name
                self.s3_bucketname = aws_resource_names_dict['s3bucketname']
        self.post_build_hook = recipe_config_dict['post-build-hook']

        # retrieve the build host section
        self.build_farm_name = build_farm_name
        build_farm_conf_dict = build_farm_config_file[self.build_farm_name]

        build_farm_type_name = build_farm_conf_dict["build-farm-type"]
        build_farm_args = build_farm_conf_dict["args"]

        build_farm_dispatch_dict = dict([(x.NAME(), x) for x in inheritors(BuildFarmHostDispatcher)])

        # create dispatcher object using class given and pass args to it
        self.build_farm_host_dispatcher = build_farm_dispatch_dict[build_farm_type_name](self, build_farm_args)

        self.build_farm_host_dispatcher.parse_args()

    def __repr__(self) -> str:
        """Print the class.

        Returns:
            String representation of the class
        """
        return "BuildConfig Object:\n" + pprint.pformat(vars(self), indent=10)

    def get_chisel_triplet(self) -> str:
        """Get the unique build-specific '-' deliminated triplet.

        Returns:
            Chisel triplet
        """
        return f"{self.DESIGN}-{self.TARGET_CONFIG}-{self.PLATFORM_CONFIG}"

    def get_build_dir_name(self) -> str:
        """Get the name of the local build directory.

        Returns:
            Name of local build directory (based on time/name).
        """
        return f"{self.launch_time}-{self.name}"

    def make_recipe(self, recipe: str) -> str:
        """Create make command for a given recipe using the tuple variables.

        Args:
            recipe: Make variables/target to run.

        Returns:
            Fully specified make command.
        """
        return f"""make {"" if self.TARGET_PROJECT is None else "TARGET_PROJECT=" + self.TARGET_PROJECT} DESIGN={self.DESIGN} TARGET_CONFIG={self.TARGET_CONFIG} PLATFORM_CONFIG={self.PLATFORM_CONFIG} {recipe}"""
