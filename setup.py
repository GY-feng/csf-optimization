import os
import platform
import shutil
import subprocess
from pathlib import Path

import numpy
from setuptools import Extension, setup
from setuptools.command.build_ext import build_ext


def bool_env(name: str) -> bool:
    return os.environ.get(name, "").strip().lower() in {"1", "true", "yes", "on"}


def find_cuda_home() -> Path:
    for name in ("CUDA_HOME", "CUDA_PATH"):
        value = os.environ.get(name)
        if value:
            return Path(value)

    nvcc = shutil.which("nvcc")
    if nvcc:
        return Path(nvcc).resolve().parent.parent

    raise RuntimeError(
        "CSF_ENABLE_CUDA=1 was set, but nvcc was not found. "
        "Install the CUDA Toolkit in WSL, or set CUDA_HOME / add nvcc to PATH."
    )


class CudaBuildExt(build_ext):
    def build_extension(self, ext):
        cuda_sources = [src for src in ext.sources if src.endswith(".cu")]
        if cuda_sources:
            ext.sources = [src for src in ext.sources if not src.endswith(".cu")]
            Path(self.build_temp).mkdir(parents=True, exist_ok=True)
            objects = []
            nvcc = str(find_cuda_home() / "bin" / ("nvcc.exe" if platform.system() == "Windows" else "nvcc"))
            cuda_arch = os.environ.get("CSF_CUDA_ARCH")

            for src in cuda_sources:
                src_path = Path(src)
                suffix = ".obj" if platform.system() == "Windows" else ".o"
                obj = Path(self.build_temp) / (src_path.stem + suffix)
                cmd = [nvcc, "-c", str(src_path), "-o", str(obj), "-std=c++11", "-O3"]
                if cuda_arch:
                    cmd.append(f"-arch={cuda_arch}")
                if platform.system() == "Windows":
                    cmd.extend(["-Xcompiler", "/MD"])
                else:
                    cmd.extend(["-Xcompiler", "-fPIC"])

                for include_dir in ext.include_dirs:
                    cmd.extend(["-I", str(include_dir)])
                for name, value in ext.define_macros:
                    if value is None:
                        cmd.append(f"-D{name}")
                    else:
                        cmd.append(f"-D{name}={value}")

                subprocess.check_call(cmd)
                objects.append(str(obj))

            ext.extra_objects = list(ext.extra_objects or []) + objects

        super().build_extension(ext)


if platform.system() == "Windows":
    openmp_args = ["/openmp", "/std:c++11"]
    openmp_linking_args = []
    openmp_macro = [("CSF_USE_OPENMP", None)]
elif platform.system() == "Linux":
    openmp_args = ["-fopenmp", "-std=c++11"]
    openmp_linking_args = ["-fopenmp"]
    openmp_macro = [("CSF_USE_OPENMP", None)]
else:
    openmp_args = ["-std=c++11"]
    openmp_linking_args = []
    openmp_macro = []


cuda_enabled = bool_env("CSF_ENABLE_CUDA")
define_macros = list(openmp_macro)
include_dirs = ["src", "src/cuda", numpy.get_include()]
library_dirs = []
libraries = []

sources = [
    "python/CSF/CSF_wrap.cxx",
    "src/c2cdist.cpp",
    "src/c2cdistSoA.cpp",
    "src/Cloth.cpp",
    "src/ClothSoA.cpp",
    "src/CSF.cpp",
    "src/Particle.cpp",
    "src/point_cloud.cpp",
    "src/Rasterization.cpp",
    "src/RasterizationSoA.cpp",
    "src/XYZReader.cpp",
]

if cuda_enabled:
    cuda_home = find_cuda_home()
    define_macros.append(("CSF_ENABLE_CUDA", None))
    include_dirs.append(str(cuda_home / "include"))
    lib_dir = cuda_home / ("lib/x64" if platform.system() == "Windows" else "lib64")
    library_dirs.append(str(lib_dir))
    libraries.append("cudart")
    sources.extend([
        "src/cuda/ClothCuda.cu",
        "src/cuda/CudaReduction.cu",
    ])


with open("README.md", encoding="utf8") as readme:
    readme_content = readme.read()


csf_module = Extension(
    name="_CSF",
    sources=sources,
    include_dirs=include_dirs,
    library_dirs=library_dirs,
    libraries=libraries,
    extra_compile_args=openmp_args,
    extra_link_args=openmp_linking_args,
    define_macros=define_macros,
)


setup(
    name="cloth_simulation_filter",
    version="1.1.7",
    author="Jianbo Qi",
    url="http://ramm.bnu.edu.cn/projects/CSF/",
    long_description=readme_content,
    long_description_content_type="text/markdown",
    install_requires=["numpy", "laspy"],
    maintainer="Jianbo Qi",
    maintainer_email="jianboqi@126.com",
    license="Apache-2.0",
    keywords="LiDAR DTM DSM Classification",
    description="CSF: Ground Filtering based on Cloth Simulation",
    package_dir={"": "python/CSF"},
    ext_modules=[csf_module],
    py_modules=["CSF"],
    cmdclass={"build_ext": CudaBuildExt},
)
