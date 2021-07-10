CROSS_COMPILE = arm-himix100-linux-
CCFLAGS = -march=armv7-a -mfpu=neon-vfpv4 -funsafe-math-optimizations
LDPATH = /opt/hisi-linux/x86-arm/arm-himix100-linux/target/usr/app/lib

CC  = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++

LDFLAGS = -pthread -llocalsdk -l_hiae -livp -live -lmpi -lmd -l_hiawb -lisp -lsecurec -lsceneauto -lVoiceEngine -lupvqe -l_hidehaze -l_hidrc -l_hildci -ldnvqe -lsns_f22 -lpaho-mqtt3c -lyyjson -lrtspserver -lstdc++

OUTPUT = ./bin
LIBDIR = ./lib

all: mkdirs mjsxj02hl

mjsxj02hl: ./mjsxj02hl.c update-libs build-libs install-libs objects
	$(CC) $(CCFLAGS) -L$(LDPATH) ./mjsxj02hl.c $(OUTPUT)/objects/*.o $(LDFLAGS) -o $(OUTPUT)/mjsxj02hl

objects: logger.o init.o configs.o inih.o osd.o video.o audio.o speaker.o alarm.o night.o mqtt.o rtsp.o

update-libs:
	git pull --recurse-submodules
	git submodule update --remote --recursive

build-libs: libyyjson.so libpaho-mqtt3c.so librtspserver.so

install-libs:
	-cp -arf $(LIBDIR)/. $(LDPATH)

libyyjson.so:
	cmake -S./yyjson -B$(OUTPUT)/objects/yyjson -DCMAKE_C_COMPILER=$(CC) -DCMAKE_C_FLAGS="$(CCFLAGS)" -DCMAKE_CXX_COMPILER=$(CXX) -DCMAKE_CXX_FLAGS="$(CCFLAGS)" -DBUILD_SHARED_LIBS=ON
	make -C $(OUTPUT)/objects/yyjson
	cp -f $(OUTPUT)/objects/yyjson/libyyjson.so $(LIBDIR)/

libpaho-mqtt3c.so:
	cmake -S./mqtt/paho.mqtt.c -B$(OUTPUT)/objects/paho.mqtt.c -DCMAKE_C_COMPILER=$(CC) -DCMAKE_C_FLAGS="$(CCFLAGS)"
	make -C $(OUTPUT)/objects/paho.mqtt.c
	cp -f $(OUTPUT)/objects/paho.mqtt.c/src/libpaho-mqtt3c.so $(LIBDIR)/
	ln -s libpaho-mqtt3c.so $(LIBDIR)/libpaho-mqtt3c.so.1

librtspserver.so:
	make -C ./rtsp

logger.o: ./logger/logger.c
	$(CC) $(CCFLAGS) -c ./logger/logger.c -o $(OUTPUT)/objects/logger.o

configs.o: ./configs/configs.c
	$(CC) $(CCFLAGS) -c ./configs/configs.c -o $(OUTPUT)/objects/configs.o

inih.o: ./configs/inih/ini.c
	$(CC) $(CCFLAGS) -c ./configs/inih/ini.c -o $(OUTPUT)/objects/inih.o

init.o: ./localsdk/init.c
	$(CC) $(CCFLAGS) -c ./localsdk/init.c -o $(OUTPUT)/objects/init.o

osd.o: ./localsdk/osd/osd.c
	$(CC) $(CCFLAGS) -c ./localsdk/osd/osd.c -o $(OUTPUT)/objects/osd.o

video.o: ./localsdk/video/video.c
	$(CC) $(CCFLAGS) -c ./localsdk/video/video.c -o $(OUTPUT)/objects/video.o

audio.o: ./localsdk/audio/audio.c
	$(CC) $(CCFLAGS) -c ./localsdk/audio/audio.c -o $(OUTPUT)/objects/audio.o

speaker.o: ./localsdk/speaker/speaker.c
	$(CC) $(CCFLAGS) -c ./localsdk/speaker/speaker.c -o $(OUTPUT)/objects/speaker.o

alarm.o: ./localsdk/alarm/alarm.c
	$(CC) $(CCFLAGS) -c ./localsdk/alarm/alarm.c -o $(OUTPUT)/objects/alarm.o

night.o: ./localsdk/night/night.c
	$(CC) $(CCFLAGS) -c ./localsdk/night/night.c -o $(OUTPUT)/objects/night.o

mqtt.o: ./mqtt/mqtt.c
	$(CC) $(CCFLAGS) -c ./mqtt/mqtt.c -o $(OUTPUT)/objects/mqtt.o

rtsp.o: ./rtsp/rtsp.c
	$(CC) $(CCFLAGS) -c ./rtsp/rtsp.c -o $(OUTPUT)/objects/rtsp.o

mkdirs: clean
	-mkdir -p $(LIBDIR)
	-mkdir -p $(OUTPUT)/objects
	-mkdir -p $(OUTPUT)/objects/yyjson
	-mkdir -p $(OUTPUT)/objects/paho.mqtt.c
	-make BUILD_DIR OUTPUT="../$(OUTPUT)" LIBDIR="../$(LIBDIR)" -C ./rtsp

clean:
	-make clean OUTPUT="../$(OUTPUT)" LIBDIR="../$(LIBDIR)" -C ./rtsp
	-make clean -C $(OUTPUT)/objects/paho.mqtt.c
	-make clean -C $(OUTPUT)/objects/yyjson
	-rm -rf $(OUTPUT)/*
	-rm -rf $(LIBDIR)/*
