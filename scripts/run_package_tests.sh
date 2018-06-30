rm -rf tests/peanut_launcher
python setup.py build
python setup.py install
pytest --rootdir=tests
pip uninstall -y peanut_launcher
cp -r peanut_launcher tests
