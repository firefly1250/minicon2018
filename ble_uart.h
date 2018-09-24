/*
    Video: https://www.youtube.com/watch?v=oCMOYS71NIU
    Based on Neil Kolban example for IDF: https://github.com/nkolban/esp32-snippets/blob/master/cpp_utils/tests/BLE%20Tests/SampleNotify.cpp
    Ported to Arduino ESP32 by Evandro Copercini

   Create a BLE server that, once we receive a connection, will send periodic notifications.
   The service advertises itself as: 6E400001-B5A3-F393-E0A9-E50E24DCCA9E
   Has a characteristic of: 6E400002-B5A3-F393-E0A9-E50E24DCCA9E - used for receiving data with "WRITE" 
   Has a characteristic of: 6E400003-B5A3-F393-E0A9-E50E24DCCA9E - used to send data with  "NOTIFY"

   The design of creating the BLE server is:
   1. Create a BLE Server
   2. Create a BLE Service
   3. Create a BLE Characteristic on the Service
   4. Create a BLE Descriptor on the characteristic
   5. Start the service.
   6. Start advertising.

   In this example rxValue is the data received (only accessible inside that function).
   And txValue is the data to be sent, in this example just a byte incremented every second. 
*/
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

uint8_t received[] = {127,127,80,30,127,0};

class BleUart : public BLECharacteristicCallbacks, public BLEServerCallbacks{
    BLEServer *pServer = nullptr;
    BLECharacteristic * pTxCharacteristic;
    bool deviceConnected = false;
    bool oldDeviceConnected = false;

    static constexpr char* SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr char* CHARACTERISTIC_UUID_RX = "6E400002-B5A3-F393-E0A9-E50E24DCCA9E";
    static constexpr char* CHARACTERISTIC_UUID_TX = "6E400003-B5A3-F393-E0A9-E50E24DCCA9E";

public:
    void Setup(){
        // Create the BLE Device
        BLEDevice::init("titanXX");

        // Create the BLE Server
        pServer = BLEDevice::createServer();
        pServer->setCallbacks(this);

        // Create the BLE Service
        BLEService *pService = pServer->createService(SERVICE_UUID);

        // Create a BLE Characteristic
        pTxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_TX,BLECharacteristic::PROPERTY_NOTIFY);
        pTxCharacteristic->addDescriptor(new BLE2902());
        BLECharacteristic * pRxCharacteristic = pService->createCharacteristic(CHARACTERISTIC_UUID_RX,BLECharacteristic::PROPERTY_WRITE);
        pRxCharacteristic->setCallbacks(this);

        // Start the service
        pService->start();

        // Start advertising
        pServer->getAdvertising()->start();
    }

    void onConnect(BLEServer* pServer) override{
      deviceConnected = true;
    }

    void onDisconnect(BLEServer* pServer) override{
      deviceConnected = false;
    }
    void onWrite(BLECharacteristic *pCharacteristic) override{
      std::string rxValue = pCharacteristic->getValue();
      if(rxValue.length() == 6){
          for(int i=0;i<6;i++) received[i]=(uint8_t)rxValue[i];
      }
      //pTxCharacteristic->setValue((uint8_t*)(rxValue.c_str()), rxValue.length());
      //pTxCharacteristic->notify();
    }

    int counter=0;
    void Loop(){
        
        /*
        if (deviceConnected) {
            if(counter%100 == 0){
                counter=0;
                //int x = received[0];
                //int y = received[1];
                char buf[10];
                //int n = sprintf(buf,"%d,%d;",x,y);
                int n = sprintf(buf,"%d;",(int)(((int)received[4]-127)/127.0f*50.0f));
                //pTxCharacteristic->setValue(std::to_string(x)+","+std::to_string(y)+"\n");
                pTxCharacteristic->setValue((uint8_t*)buf,n);
                pTxCharacteristic->notify();
            }
            counter++;
		    //delay(100); // bluetooth stack will go into congestion, if too many packets are sent
	    }
        */

        // disconnecting
        if (!deviceConnected && oldDeviceConnected) {
            delay(500); // give the bluetooth stack the chance to get things ready
            pServer->startAdvertising(); // restart advertising
            oldDeviceConnected = deviceConnected;
        }
        // connecting
        if (deviceConnected && !oldDeviceConnected) {
    		// do stuff here on connecting
            oldDeviceConnected = deviceConnected;
        }
    }
};