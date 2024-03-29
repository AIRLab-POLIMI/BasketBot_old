#include "ch.h"
#include "hal.h"

#include <r2p/common.hpp>
#include <r2p/Middleware.hpp>
#include <r2p/Node.hpp>
#include <r2p/Topic.hpp>
#include <r2p/Publisher.hpp>
#include <r2p/Subscriber.hpp>
#include <r2p/Mutex.hpp>
#include <r2p/NamingTraits.hpp>
#include <r2p/Bootloader.hpp>
#include <r2p/transport/DebugTransport.hpp>
#include <r2p/transport/RTCANTransport.hpp>

#include <r2p/node/led.hpp>
#include <r2p/msg/motor.hpp>

#include <cstdio>
#include <cstdlib>
#include <cstring>

#if R2P_USE_BRIDGE_MODE
#define R2P_MODULE_NAME "IMUGW"
#else
#define R2P_MODULE_NAME "IMU1"
#endif

#if R2P_USE_BRIDGE_MODE
#define DEBUGTRA    1
#else
#define DEBUGTRA    0
#endif

#ifndef RTCANTRA
#define RTCANTRA    1
#endif

struct TestMsg: public r2p::Message {
	long id;
	long value;
}R2P_PACKED;


#define BOOT_STACKLEN   1024

#if R2P_USE_BRIDGE_MODE
enum {PUBSUB_BUFFER_LENGTH = 16};
r2p::Middleware::PubSubStep pubsub_buf[PUBSUB_BUFFER_LENGTH];
#endif

static WORKING_AREA(wa_info, 1024);

r2p::Middleware r2p::Middleware::instance(R2P_MODULE_NAME, "BOOT_"R2P_MODULE_NAME
#if R2P_USE_BRIDGE_MODE
		, pubsub_buf, PUBSUB_BUFFER_LENGTH
#endif
		);

/*
 static WORKING_AREA(wa1, 1024);
 static WORKING_AREA(wa2, 1024);
 static WORKING_AREA(wa3, 1024);
 */

#if DEBUGTRA
static char dbgtra_namebuf[64];
static r2p::DebugTransport dbgtra("SD2", reinterpret_cast<BaseChannel *>(&SD2),
		dbgtra_namebuf);
static WORKING_AREA(wa_rx_dbgtra, 1024);
static WORKING_AREA(wa_tx_dbgtra, 1024);
#endif // DEBUGTRA
#if RTCANTRA
static r2p::RTCANTransport rtcantra(RTCAND1);
static RTCANConfig rtcan_config = { 1000000, 100, 60 };
#endif // RTCANTRA
size_t num_msgs = 0;
systime_t start_time, cur_time;

msg_t Thread1(void *) {

	r2p::Node node("Node1");
	r2p::Publisher<TestMsg> pub1;

	node.advertise(pub1, "test");

	chThdSleepMilliseconds(1000);
	start_time = chTimeNow();

	for (;;) {
		chThdSleepMilliseconds(250);
		cur_time = chTimeNow();
		TestMsg *msgp;
		if (pub1.alloc(msgp)) {
			msgp->id = 0xDEADBEEF;
			msgp->value = 0x1337B00B;
			if (!pub1.publish(*msgp)) {
				chSysHalt();
			}
			palTogglePad(LED1_GPIO, LED1);
		} else {
			palTogglePad(LED1_GPIO, LED4);
		}
	}
	return CH_SUCCESS;
}

bool callback_2(const TestMsg &msg) {
	(void) msg;
	palTogglePad(LED1_GPIO, LED2);
	++num_msgs;
	return true;
}

static unsigned remote_msg_count = 0;

bool callback_3(const TestMsg &msg) {
	(void) msg;
	++remote_msg_count;
	palTogglePad(LED1_GPIO, LED3);
	return true;
}

msg_t Thread2(void *) {
	r2p::Node node("Node2");
	r2p::Subscriber<TestMsg, 5> sub2(callback_2);

	node.subscribe(sub2, "test");

	for (;;) {
		if (!node.spin()) {
			palTogglePad(LED1_GPIO, LED4);
		}
	}
	return CH_SUCCESS;
}

msg_t Thread3(void *) {
	r2p::Node node("Node3");
	r2p::Subscriber<TestMsg, 5> sub3(callback_3);

	node.subscribe(sub3, "asdf");

	for (;;) {
		if (!node.spin()) {
			palTogglePad(LED1_GPIO, LED4);
		}
	}
	return CH_SUCCESS;
}

extern "C" {
int main(void) {

	halInit();
	chSysInit();

	palClearPad(LED1_GPIO, LED1);
	palClearPad(LED2_GPIO, LED2);
	palClearPad(LED3_GPIO, LED3);
	palClearPad(LED4_GPIO, LED4);
	chThdSleepMilliseconds(500);
	palSetPad(LED1_GPIO, LED1);
	palSetPad(LED2_GPIO, LED2);
	palSetPad(LED3_GPIO, LED3);
	palSetPad(LED4_GPIO, LED4);

	void *boot_stackp = NULL;
	if (r2p::Middleware::is_bootloader_mode()) {
		uint8_t *stackp = new uint8_t[BOOT_STACKLEN + sizeof(stkalign_t)];
		R2P_ASSERT(stackp != NULL);
		stackp += (sizeof(stkalign_t) - reinterpret_cast<uintptr_t>(stackp)) % sizeof(stkalign_t);
		boot_stackp = stackp;
	}

	r2p::Middleware::instance.initialize(wa_info, sizeof(wa_info), r2p::Thread::LOWEST, boot_stackp, BOOT_STACKLEN,
			r2p::Thread::LOWEST);

#if DEBUGTRA
	sdStart(&SD2, NULL);
	dbgtra.initialize(wa_rx_dbgtra, sizeof(wa_rx_dbgtra), r2p::Thread::LOWEST + 11,
			wa_tx_dbgtra, sizeof(wa_tx_dbgtra), r2p::Thread::LOWEST + 10);
#endif

#if RTCANTRA
	rtcantra.initialize(rtcan_config);
#endif

	r2p::Middleware::instance.start();

	uint32_t led = 3;
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledpub_node, &led);
	r2p::Thread::create_heap(NULL, THD_WA_SIZE(512), NORMALPRIO, r2p::ledsub_node, NULL);

	for (;;) {
		palTogglePad(LED1_GPIO, LED1);
		if (r2p::Middleware::is_bootloader_mode()) {
			r2p::Thread::sleep(r2p::Time::ms(100));

		} else {
			r2p::Thread::sleep(r2p::Time::ms(500));
		}
	}
	return CH_SUCCESS;
}
}
