pip uninstall peanut_launcher
pip install git+https://github.com/tianhaoz95/peanut_launcher.git
rm -rf tests/peanut_launcher
pytest --rootdir=tests
