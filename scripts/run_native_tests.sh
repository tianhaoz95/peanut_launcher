rm -rf ../tests/peanut_launcher
cp -r ../peanut_launcher ../tests
pytest --rootdir=../tests
