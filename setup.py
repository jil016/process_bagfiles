from setuptools import find_packages
from setuptools import setup


def main():
    setup(
        name="rosbag_tools",
        version="1.0",
        packages=find_packages(),
        install_requires=[],
        author="Jinzhao Li, thuljz@gmail.com",
    )


if __name__ == "__main__":
    main()
