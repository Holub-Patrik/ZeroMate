cmake --build --preset ninja_clang_release
if [ -d "output_2" ]; then
	rm -r output_2
fi

cp -r output output_2
cp examples/23-RemoteI2C/peripherals-master.json output/peripherals.json
cp examples/23-RemoteI2C/peripherals-slave.json output_2/peripherals.json
