from setuptools import setup
from setuptools import find_packages

setup(name='peanut_launcher',
      version='1.0.0',
      description='Easy launch for ROS project',
      author='Jason Chiau',
      author_email='tianhaoz@umich.edu',
      url='https://github.com/tianhaoz95',
      download_url='https://github.com/tianhaoz95',
      license='MIT',
      install_requires=['six>=1.9.0'],
      extras_require={
          'tests': ['pytest']
      },
      classifiers=[
          'Development Status :: 1 - Alpha',
          'Intended Audience :: Developers',
          'Intended Audience :: Education',
          'Intended Audience :: Science/Research',
          'License :: OSI Approved :: MIT License',
          'Programming Language :: Python :: 2',
          'Programming Language :: Python :: 2.7',
          'Programming Language :: Python :: 3',
          'Programming Language :: Python :: 3.6',
          'Topic :: Software Development :: Libraries',
          'Topic :: Software Development :: Libraries :: Python Modules'
      ],
      packages=find_packages())
