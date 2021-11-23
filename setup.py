from glob import glob
import os
import re
import subprocess
import sys

from setuptools import Extension, find_packages, setup
from setuptools.command.build_ext import build_ext as _build_ext


class getPybindInclude(object):
    """Helper class to determine the pybind11 include path
    The purpose of this class is to postpone importing pybind11
    until it is actually installed, so that the ``get_include()``
    method can be invoked.
    https://github.com/pybind/python_example/blob/master/setup.py
    """

    def __init__(self, user=False):
        self.user = user

    def __str__(self):
        import pybind11
        return pybind11.get_include(self.user)


def getIncludes():
    return [
        'python/kaiju/include',
        getPybindInclude(),
        getPybindInclude(user=True),
    ]


def getVersion():
    with open("python/kaiju/__version__.py", "r") as f:
        lines = f.readlines()
    for line in lines:
        if line.startswith("__version__"):
            v = line.split("=")[-1].strip().strip('"').strip("'")
            return v


def getSources():
    return [
        'src/cKaiju.cpp',
        'src/robot.cpp',
        'src/robotGrid.cpp',
        'src/utils.cpp',
        'src/target.cpp',
        'src/fiducial.cpp',
        'src/gfa.cpp'
    ]


extra_compile_args = ["--std=c++11", "-fPIC", "-v", "-O3"]
extra_link_args = []
if sys.platform == 'darwin':
    extra_compile_args += ['-stdlib=libc++', '-mmacosx-version-min=10.9']
    extra_link_args = ["-v", '-mmacosx-version-min=10.9']

    from distutils import sysconfig
    vars = sysconfig.get_config_vars()
    vars['LDSHARED'] = vars['LDSHARED'].replace('-bundle', '-dynamiclib')

module = Extension(
    'kaiju.cKaiju',
    include_dirs=getIncludes(),
    extra_compile_args=extra_compile_args,
    extra_link_args=extra_link_args,
    sources=getSources()
)


# We need a custom build extension class because we need to inspect the
# coordio library after setup_requires has installed id.
class build_ext(_build_ext):

    def run(self):
        super().run()

        # On macOS we need to rename the path to the libcoordio library in cKaiju
        # after it has been linked.
        if sys.platform == 'darwin':
            libs = list(glob('python/kaiju/cKaiju*.so'))
            if len(libs) == 0:
                libs = list(glob('build/**/kaiju/cKaiju*.so'))
                if len(libs) == 0:
                    return

            kaiju_lib = libs[0]
            otool_data = subprocess.run(f'otool -L {kaiju_lib}',
                                        capture_output=True, shell=True)

            reg = re.compile(r'\n\s*(.+libcoordio.+?)\s+.+\n', re.MULTILINE)
            libcoordio_path_current = reg.search(otool_data.stdout.decode()).groups()[0]

            from coordio import libcoordio
            libcoordio_path = libcoordio.__file__

            subprocess.run(f'install_name_tool -change {libcoordio_path_current} '
                           f'{libcoordio_path} {kaiju_lib}', shell=True)

    def finalize_options(self):
        _build_ext.finalize_options(self)

        # JSG: this is a bit of a hack but what we need is not very standard.
        # We want to link against libcoordio which is located in the
        # site-packages/coordio directory. We need to provide the full path
        # (since Python calls it libcoorio.something.architecture.so) and
        # we also need to set the rpath in cKaiju so that it knows how to
        # find it in runtime without having to override LD_LIBRARY_PATH.
        # We also need to tell the linker where to find the library with -L.
        # Finally, we need to tell the compiler where the headers for
        # libcoordio are.

        import coordio

        coordio_base = os.path.abspath(os.path.dirname(coordio.__file__))

        coordio_include = os.path.join(coordio_base, "include")
        self.extensions[0].include_dirs.append(coordio_include)

        from coordio import libcoordio
        libcoordio_path = libcoordio.__file__

        self.extensions[0].extra_link_args += [
            libcoordio_path,  # This is equivalent to -l but with full path.
            '-Wl,-rpath,' + coordio_base,
            '-L' + coordio_base]


def runSetup(packages, requirements):
    setup(
        name="sdss-kaiju",
        version=getVersion(),
        license="BSD3",
        author="Conor Sayres",
        author_email="csayres@uw.edu",
        description="Collision Avoidance for SDSS-V Positioners",
        long_description=open('README.rst').read(),
        packages=packages,
        cmdclass={'build_ext': build_ext},
        package_dir={'': 'python'},
        package_data={'kaiju': ['python/kaiju/include/*']},
        include_package_data=True,
        url="https://github.com/sdss/kaiju",
        keywords="astronomy software",
        ext_modules=[module],
        install_requires=requirements,
        setup_requires=[
            "sdss-coordio>=1.1.2",
            "pybind11>=2.2.4"
        ],
        classifiers=[
            'Intended Audience :: Science/Research',
            'License :: OSI Approved :: BSD License',
            'Natural Language :: English',
            'Operating System :: OS Independent',
            'Programming Language :: Python',
        ]
    )


requirementsFile = os.path.join(os.path.dirname(__file__), "requirements.txt")
requirements = [line.strip() for line in open(requirementsFile)]
packages = find_packages(where="python")
print("found packages", packages)
runSetup(packages, requirements)
