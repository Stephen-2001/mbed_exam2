#include "accelerometer_handler.h"
#include "config.h"
#include "magic_wand_model_data.h"
#include "mbed.h"
#include "mbed_rpc.h"
#include "stm32l475e_iot01_accelero.h"
#include "uLCD_4DGL.h"
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include <cmath>

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];
uLCD_4DGL uLCD(D1, D0, D2);
// GLOBAL VARIABLES
WiFiInterface *wifi;

volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;
const char* topic = "Mbed";
bool gesture_mode = false;
bool angle_mode = false;
BufferedSerial pc(USBTX, USBRX);

void angle_detect_RPC(Arguments *in, Reply *out);
void gesture_detect_RPC(Arguments *in, Reply *out);
void gesture_detect();
void gesture_UI();
void angle_detect();
int PredictGesture(float* output);
void selected_mode();
void messageArrived(MQTT::MessageData& md);
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
void close_mqtt();
void publish_present_gesture(MQTT::Client<MQTTNetwork, Countdown>* client);
int confirm_shape();
bool angle_check(int index);
#define pi 3.1415926

RPCFunction rpc3(&gesture_detect_RPC, "g");  // /g/run

DigitalOut myled1(LED1);	// gesture_UI mode
DigitalOut myled2(LED2);	// angle_detect mode
DigitalOut myled3(LED3);	// initialize
InterruptIn button(USER_BUTTON);
Thread mqtt_thread(osPriorityHigh);
Thread t_angle_detect;
Thread t_gesture_UI;
Thread t_gesture_detect;

Thread mqtt_thread_present_angle(osPriorityHigh);
EventQueue mqtt_queue_present_angle;
EventQueue mqtt_queue;
EventQueue queue;
bool shape = false;
bool publish = false;
bool selected = true;
bool detect = false;
int publish_count = 0;
int idR[32] = {0};
int indexR = 0;
int angle_threshold = 30;
double overthreshold[10] = {0};
int num_overthreshold = 0;
int16_t data_x[32] = {0};
int16_t data_y[32] = {0};
int16_t data_z[32] = {0};
int16_t data_total[3] = {0};
int16_t ref_data[3] = {0};
int sequence[32] = {0};
int number = 0;
int gesture_id;

void uLCD_print(){
	selected = false;
  gesture_mode = false;

	uLCD.cls();
	uLCD.locate(1, 2);
	uLCD.text_width(2); //4X size text
	uLCD.text_height(3);
	uLCD.color(RED);
	uLCD.printf("SELECTED!");

	uLCD.text_width(1); //4X size text
	uLCD.text_height(1);
	uLCD.color(GREEN);
	uLCD.locate(0, 10);
	uLCD.printf("angle_threshold = %d\n", angle_threshold);
}

void messageArrived(MQTT::MessageData& md) {
	MQTT::Message &message = md.message;
	char msg[300];
	sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
	printf(msg);
	ThisThread::sleep_for(1000ms);
	char payload[300];
	sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
	printf(payload);
	++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {
	message_num++;
	MQTT::Message message;
	char buff[100];
	
	if (selected) {
		uLCD_print();	
		sprintf(buff, "angle_threshold = %d degree\n", angle_threshold);
	}
	else {
		sprintf(buff, "present_angle: %f degree", overthreshold[num_overthreshold]);
	}
	
	message.qos = MQTT::QOS0;
	message.retained = false;
	message.dup = false;
	message.payload = (void*) buff;
	message.payloadlen = strlen(buff) + 1;
	int rc = client->publish(topic, message);

	printf("rc:  %d\r\n", rc);
	printf("Puslish message: %s\r\n", buff);
}

void publish_present_gesture(MQTT::Client<MQTTNetwork, Countdown>* client) {
	message_num++;
	MQTT::Message message;
	char buff[100];
	if (num_overthreshold >= 10) return ;
	sprintf(buff, "present_angle: %f\n", overthreshold[num_overthreshold]);

	message.qos = MQTT::QOS0;
	message.retained = false;
	message.dup = false;
	message.payload = (void*) buff;
	message.payloadlen = strlen(buff) + 1;
	int rc = client->publish(topic, message);

	printf("rc:  %d\r\n", rc);
	printf("Puslish message: %s\r\n", buff);
}
void close_mqtt() {
  closed = true;
}

void gesture_detect_RPC(Arguments *in, Reply *out) {
  detect = true;
  gesture_mode = true;
  printf("detect = %d, gesture_mode = %d\n", detect, gesture_mode);
  t_gesture_detect.start(&gesture_detect);
  t_gesture_UI.start(&gesture_UI);
}
void gesture_detect(){
  printf("LED1 should be turn on!!!");
  myled1 = 1;
  myled3 = 1;
  ThisThread::sleep_for(3s);
  BSP_ACCELERO_AccGetXYZ(ref_data);
  ThisThread::sleep_for(1s);
  myled3 = 0;
  number = 0;

  while(1) {
    if (!shape){  // when TF model sense a gesture, the shape variable will be true
      BSP_ACCELERO_AccGetXYZ(data_total);
      if (number >= 32) number = number % 32;
      data_x[number] = data_total[0];
      data_y[number] = data_total[1];
      data_z[number] = data_total[2];
      number++;
    }
    else {
      for (int i=0; i<32; i++) {
        printf("data_x[%d] = %d, data_y[%d] = %d, data_z[%d] = %d\n", i, data_x[i], i, data_y[i], i, data_z[i]);
      }
      
      bool shape_trigger = confirm_shape();
      if (!shape_trigger) shape = false;
      publish_count++;
      if (publish_count == 10) {
        publish = true;
        publish_count = 0;
      }
    }
    ThisThread::sleep_for(50ms);
  }

  return ;
}

int confirm_shape() {
  // after confirm, the trigger will be false
  bool trigger = true;
  int count = 0;
  int i;
  for (i=0; i<32; i++) {
    sequence[i] = angle_check(i);
    if(sequence[i]) count++;
  }
  if (count>=20) {
    gesture_id = 0; // circle
  }
  else {
    gesture_id = 1; // slope
  }
  trigger = false;
  return trigger;
}

bool angle_check(int index){
  bool check = false; // not over the threshold angle
  int threshold_angle = 30;
  double a, b, c, cos, theta;
  if (index==0) {
    a = sqrt(pow(ref_data[0],2) + pow(ref_data[1],2) + pow(ref_data[2],2));
    b = sqrt(pow(data_x[0],2) + pow(data_y[0],2) + pow(data_z[0],2));
    c = sqrt(pow((data_x[0]-ref_data[0]),2) + pow((data_y[0]-ref_data[1]),2) + pow((data_z[0]-ref_data[2]),2));
    cos = (a*a+b*b-c*c) / (2*a*b);
    theta = acos((pow(a,2) + pow(b,2) - pow(c,2)) / (2 * a * b)) * 180 / pi;
  }
  else {
    a = sqrt(pow(data_x[index],2) + pow(data_y[index],2) + pow(data_z[index],2));
    b = sqrt(pow(data_x[index-1],2) + pow(data_y[index-1],2) + pow(data_z[index-1],2));
    c = sqrt(pow((data_x[index]-data_x[index-1]),2) + pow((data_y[index]-data_y[index-1]),2) + pow((data_z[index]-data_z[index-1]),2));
    cos = (a*a+b*b-c*c) / (2*a*b);
    theta = acos((pow(a,2) + pow(b,2) - pow(c,2)) / (2 * a * b)) * 180 / pi;
  }
  if (theta > threshold_angle) check = true;
  return check;
}

void gesture_UI() {
  // Whether we should clear the buffer next time we fetch data
  // myled1 = 1;
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;

  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
    "Model provided is schema version %d not equal "
    "to supported version %d.",
    model->version(), TFLITE_SCHEMA_VERSION);
    return ;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
    model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return ;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return ;
  }

  error_reporter->Report("Set up successful...\n");

  uLCD.printf("angle_threshold = %d degree\n", angle_threshold);
  while (true) {
    if (gesture_mode) {
      // Attempt to read new data from the accelerometer
      got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                  input_length, should_clear_buffer);

      // If there was no new data,
      // don't try to clear the buffer again and wait until next time
      if (!got_data) {
        should_clear_buffer = false;
        continue;
      }

      // Run inference, and report any error
      TfLiteStatus invoke_status = interpreter->Invoke();
      if (invoke_status != kTfLiteOk) {
        error_reporter->Report("Invoke failed on index: %d\n", begin_index);
        continue;
      }

      // Analyze the results to obtain a prediction
      gesture_index = PredictGesture(interpreter->output(0)->data.f);

      // Clear the buffer next time we read data
      should_clear_buffer = gesture_index < label_num;

      // Produce an output
      if (gesture_index < label_num) {
        // uLCD.cls();
        // uLCD.printf("angle_threshold = %d degree\n", angle_threshold);
        error_reporter->Report(config.output_message[gesture_index]);
        shape = true;
        printf("shape = %d\n", shape);
      }
    } 
  }
}

int main() {
	BSP_ACCELERO_Init();

  wifi = WiFiInterface::get_default_instance();
  if (!wifi) {
    printf("ERROR: No WiFiInterface found.\r\n");
    return -1;
  }
  printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
  int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
  if (ret != 0) {
    printf("\nConnection error: %d\r\n", ret);
    return -1;
  }
  NetworkInterface* net = wifi;
  MQTTNetwork mqttNetwork(net);
  MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

  //TODO: revise host to your IP
  const char* host = "172.20.10.5";
  printf("Connecting to TCP network...\r\n");

  SocketAddress sockAddr;
  sockAddr.set_ip_address(host);
  sockAddr.set_port(1883);

  printf("address is %s/%d\r\nSuccessfully connected!\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

  int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
  if (rc != 0) {
		printf("Connection error.");
		return -1;
  }
  printf("Successfully connected!\r\n");

  MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
  data.MQTTVersion = 3;
  data.clientID.cstring = "Mbed";

  if ((rc = client.connect(data)) != 0){
    printf("Fail to connect MQTT\r\n");
  }
  if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0){
    printf("Fail to subscribe\r\n");
  }

  mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
  mqtt_thread_present_angle.start(callback(&mqtt_queue_present_angle, &EventQueue::dispatch_forever));
  t_gesture_detect.start(callback(&queue, &EventQueue::dispatch_forever));
	char buf[256], outbuf[256];

	FILE *devin = fdopen(&pc, "r");
	FILE *devout = fdopen(&pc, "w");

	while (1) {
    button.rise(mqtt_queue.event(&publish_message, &client));
		if (publish) {
      ThisThread::sleep_for(1s);
			for (num_overthreshold=0; num_overthreshold<10; num_overthreshold++) {
				mqtt_queue_present_angle.call(&publish_present_gesture, &client);
				ThisThread::sleep_for(50ms);
			}
 		}
		memset(buf, 0, 256);      // clear buffer
		for(int i=0; i<255; i++) {
			char recv = fgetc(devin);
			if (recv == '\r' || recv == '\n') {
				printf("\r\n");
				break;
			}
			buf[i] = fputc(recv, devout);
		}
		RPC::call(buf, outbuf);
		printf("%s\r\n", outbuf);
	}

  int num = 0;
  while (num != 5) {
    client.yield(100);
    ++num;
  }

  while (1) {
    if (closed) break;
    client.yield(500);
    ThisThread::sleep_for(500ms);
  }

  printf("Ready to close MQTT Network......\n");

  if ((rc = client.unsubscribe(topic)) != 0) {
          printf("Failed: rc from unsubscribe was %d\n", rc);
  }
  if ((rc = client.disconnect()) != 0) {
  printf("Failed: rc from disconnect was %d\n", rc);
  }

  mqttNetwork.disconnect();
  printf("Successfully closed!\n");

  return 0;
}
// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

