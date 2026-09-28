SKIP_SHARED_LIBS = OFF

CROSS_COMPILE = arm-himix100-linux-
CCFLAGS = -march=armv7-a -mfpu=neon-vfpv4 -funsafe-math-optimizations -I./include
LDPATH = /opt/hisi-linux/x86-arm/arm-himix100-linux/target/usr/app/lib

CC  = $(CROSS_COMPILE)gcc
CXX = $(CROSS_COMPILE)g++

LDFLAGS = -Wl,--gc-sections -pthread -l_hiae -livp -live -lmpi -lmd -l_hiawb -lisp -lsecurec -lVoiceEngine -lupvqe -l_hidehaze -l_hidrc -l_hildci -ldnvqe -lrtspserver -lstdc++ -ldl -lrt

# yyjson is linked statically, trimmed to what we use (small MQTT objects, one
# parsed command): no JSON pointer/patch utils, no incremental reader, no
# non-standard JSON, no fast float tables (~95 KB; the few floats go through
# snprintf/strtod). Sections let --gc-sections drop the unused functions.
YYJSON_FLAGS = -Os -ffunction-sections -fdata-sections \
	-DYYJSON_DISABLE_UTILS=1 -DYYJSON_DISABLE_INCR_READER=1 \
	-DYYJSON_DISABLE_NON_STANDARD=1 -DYYJSON_DISABLE_FAST_FP_CONV=1

# paho (synchronous MQTTClient, no SSL) is linked statically as well.
# HIGH_PERFORMANCE drops paho's tracing and its heap tracking, which records
# every allocation in a tree (we use neither). Its system deps (dl, rt) are
# added to LDFLAGS since a static archive does not carry them.
PAHO_FLAGS = -DPAHO_BUILD_SHARED=FALSE -DPAHO_BUILD_STATIC=TRUE \
	-DPAHO_HIGH_PERFORMANCE=TRUE -DPAHO_ENABLE_TESTING=FALSE -DPAHO_ENABLE_CPACK=FALSE

OUTPUT = ./bin
LIBDIR = ./lib

###############
# APPLICATION #
###############

all: mkdirs mjsxj02hl

mjsxj02hl: ./mjsxj02hl.c external-libs objects
	$(CC) $(CCFLAGS) -L$(LDPATH) ./mjsxj02hl.c $(OUTPUT)/objects/*.o $(OUTPUT)/objects/*.a -o $(OUTPUT)/mjsxj02hl $(LDFLAGS)

##############
# PVS-STUDIO #
##############

PVS_ANALYZER = GA:1,2,3
PVS_TYPE  = html

analyze:
	pvs-studio-analyzer trace -- make SKIP_SHARED_LIBS=$(SKIP_SHARED_LIBS)
	pvs-studio-analyzer analyze --compiler $(CC) --compiler $(CXX) -e bin -e /opt -e /usr -e configs/inih -e mqtt/paho.mqtt.c -e rtsp/RtspServer -e yyjson -e ipctool
	plog-converter -a $(PVS_ANALYZER) -t $(PVS_TYPE) -d V1019 -o PVS-Studio.$(PVS_TYPE) PVS-Studio.log

#################
# EXTERNAL LIBS #
#################

ifeq ($(SKIP_SHARED_LIBS), OFF)
external-libs: clean-libs mkdir-libs static-libs shared-libs install-libs
else
external-libs: clean-libs mkdir-libs static-libs install-libs
endif

clean-libs:
	-make clean OUTPUT="../$(OUTPUT)" LIBDIR="../$(LIBDIR)" -C ./rtsp
	-make clean -C $(OUTPUT)/objects/paho.mqtt.c
	-make clean -C $(OUTPUT)/objects/ipctool ; rm -f $(OUTPUT)/ipctool
	-rm -rf $(LIBDIR)/*

mkdir-libs:
	-mkdir -p $(LIBDIR)
	-mkdir -p $(OUTPUT)/objects/ipctool
	-mkdir -p $(OUTPUT)/objects/paho.mqtt.c
	-make BUILD_DIR OUTPUT="../$(OUTPUT)" LIBDIR="../$(LIBDIR)" -C ./rtsp

update-libs:
	git submodule sync --recursive
	git pull --recurse-submodules
	git submodule update --remote --recursive

static-libs: libipchw.a libpaho-mqtt3c.a
shared-libs: librtspserver.so

install-libs:
	-cp -arf $(LIBDIR)/. $(LDPATH)

libipchw.a:
	cmake -S./ipctool -B$(OUTPUT)/objects/ipctool -DCMAKE_C_COMPILER=$(CC) -DCMAKE_C_FLAGS="$(CCFLAGS)" -DBUILD_SHARED_LIBS=ON -DCMAKE_BUILD_TYPE=Release
	make -C $(OUTPUT)/objects/ipctool ipchw ipctool
	cp -f $(OUTPUT)/objects/ipctool/libipchw.a $(OUTPUT)/objects/
	cp -f $(OUTPUT)/objects/ipctool/ipctool $(OUTPUT)/

libpaho-mqtt3c.a:
	cmake -S./mqtt/paho.mqtt.c -B$(OUTPUT)/objects/paho.mqtt.c -DCMAKE_C_COMPILER=$(CC) -DCMAKE_C_FLAGS="$(CCFLAGS) -Os -ffunction-sections -fdata-sections" $(PAHO_FLAGS)
	make -C $(OUTPUT)/objects/paho.mqtt.c paho-mqtt3c-static
	cp -f $(OUTPUT)/objects/paho.mqtt.c/src/libpaho-mqtt3c.a $(OUTPUT)/objects/

librtspserver.so:
	make -C ./rtsp

#######################
# APPLICATION OBJECTS #
#######################

objects: logger.o init.o configs.o inih.o osd.o video.o audio.o speaker.o alarm.o night.o mqtt.o homeassistant.o rtsp.o sensor.o board.o jxf22_cmos.o jxf22_ctl.o scene.o yyjson.o

logger.o: ./logger/logger.c
	$(CC) $(CCFLAGS) -c ./logger/logger.c -o $(OUTPUT)/objects/logger.o

configs.o: ./configs/configs.c
	$(CC) $(CCFLAGS) -c ./configs/configs.c -o $(OUTPUT)/objects/configs.o

inih.o: ./configs/inih/ini.c
	$(CC) $(CCFLAGS) -c ./configs/inih/ini.c -o $(OUTPUT)/objects/inih.o

yyjson.o: ./yyjson/src/yyjson.c
	$(CC) $(CCFLAGS) $(YYJSON_FLAGS) -c ./yyjson/src/yyjson.c -o $(OUTPUT)/objects/yyjson.o

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

homeassistant.o: ./mqtt/homeassistant.c
	$(CC) $(CCFLAGS) -c ./mqtt/homeassistant.c -o $(OUTPUT)/objects/homeassistant.o

rtsp.o: ./rtsp/rtsp.c
	$(CC) $(CCFLAGS) -c ./rtsp/rtsp.c -o $(OUTPUT)/objects/rtsp.o

sensor.o: ./localsdk/sensor/jxf/sensor_jxf22.c
	$(CC) $(CCFLAGS) -I./localsdk/sensor/jxf -c ./localsdk/sensor/jxf/sensor_jxf22.c -o $(OUTPUT)/objects/sensor.o

board.o: ./localsdk/platform/board_mjsxj02hl.c
	$(CC) $(CCFLAGS) -c ./localsdk/platform/board_mjsxj02hl.c -o $(OUTPUT)/objects/board.o

jxf22_cmos.o: ./localsdk/sensor/jxf/jxf22_cmos.c
	$(CC) $(CCFLAGS) -I./localsdk/sensor/jxf -c ./localsdk/sensor/jxf/jxf22_cmos.c -o $(OUTPUT)/objects/jxf22_cmos.o

jxf22_ctl.o: ./localsdk/sensor/jxf/jxf22_sensor_ctl.c
	$(CC) $(CCFLAGS) -I./localsdk/sensor/jxf -c ./localsdk/sensor/jxf/jxf22_sensor_ctl.c -o $(OUTPUT)/objects/jxf22_ctl.o

scene.o: ./localsdk/scene/scene.c
	$(CC) $(CCFLAGS) -I./configs/inih -I./logger -c ./localsdk/scene/scene.c -o $(OUTPUT)/objects/scene.o

clean:
	-rm -rf $(OUTPUT)/*

mkdirs: clean
	-mkdir -p $(OUTPUT)/objects
