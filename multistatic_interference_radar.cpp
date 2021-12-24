#include "multistatic_interference_radar.h"

#include <Arduino.h>  // only for diagnostic messages via Serial

#include <WiFi.h>  // THIS WILL MAKE THE LIBRARY WORK ONLY IN STA MODE OR AP_STA MODE AS LONG AS YOU'RE CONNECTED AS A STATION, TO ANOTHER ACCESS POINT

#include "esp_wifi.h"   // IF YOU HAVE AN ESP32, THIS WILL MAKE THE LIBRARY WORK IN MULTISTATIC MODE AND SOFT AP MODE AS LONG AS YOU'VE GOT AT LEAST ONE STATION CONNECTED TO YOUR SoftAP

#include <math.h>  // testing some new improvements
int debugRadarMsg = 3;






int multistatic_interference_radar_process(int sample, transmitterData *transmitterX) { // send the RSSI signal, returns the detection level ( < 0 -> error, == 0 -> no detection, > 0 -> detection level in dBm)


  if ((transmitterX->sampleBuffer == NULL) || (transmitterX->mobileAverageBuffer == NULL) || (transmitterX->varianceBuffer == NULL)) {

    if (debugRadarMsg >= 1) {
      Serial.print("multistatic_interference_radar_process(): alarm: detected unallocated buffers: did you call bistatic_interference_radar_init() to allocate the buffers?");
    }
    return RADAR_UNINITIALIZED; // unallocated buffers
  }



  // did we receive a rest request? 

  if (transmitterX->resetRequest == 1) {
    transmitterX->resetRequest = 0;
    transmitterX->sampleBufferIndex = 0;
    transmitterX->mobileAverageBufferIndex = 0;
    transmitterX->varianceBufferIndex = 0;
    transmitterX->resetRequest = 0;
    transmitterX->sampleBufferValid = 0;
    transmitterX->mobileAverageBufferValid = 0;
    transmitterX->varianceBufferValid = 0;
    transmitterX->variance = -1;
  }


  // store a copy of the actual received sample, whatever it is, for external usage
  transmitterX->latestReceivedSample = sample; 
  
  // now doing the minimum_RSSI test and updating the internat structure status
  if (sample >= transmitterX->minimum_RSSI) {
    transmitterX->currentRSSI = sample;
  }
  
  if (sample < transmitterX->minimum_RSSI) {
    
    if (debugRadarMsg >= 4) {
      Serial.print("multistatic_interference_radar_process(): warning: RSSI under the minimum RSSI threshold detected for the current instance");
    }
    if (transmitterX->sampleBufferValid == 1) {
      sample = transmitterX->mobileAverage;
    }
    if (transmitterX->sampleBufferValid == 0) {
      if (transmitterX->currentRSSI == 0) {
        transmitterX->currentRSSI = sample;
      }
      sample = transmitterX->currentRSSI;
    }
  }
  

  transmitterX->sampleBuffer[transmitterX->sampleBufferIndex] = sample;
  transmitterX->sampleBufferIndex++;
  if ( transmitterX->sampleBufferIndex >= transmitterX->sampleBufferSize ) { // circular buffer, rewinding the index, if the buffer has been filled at least once, then we may start processing valid data
    transmitterX->sampleBufferIndex = 0;
    transmitterX->sampleBufferValid = 1;
  }
  
  if (transmitterX->sampleBufferValid >= 1) {
    // filling in the mobile average data buffer
    // the mobile average can be re-calculated even on a full sampleBufferSize set of valid samples, I see no problem in terms of computational load
    // calculating the current mobile average now.  the sampleBufferIndex points now to the oldest sample
    transmitterX->mobileAverageTemp = 0;
    int mobilePointer = 0;
    for (int mobileAverageSampleIndex = 0; mobileAverageSampleIndex < transmitterX->mobileAverageFilterSize; mobileAverageSampleIndex++) {
      mobilePointer = transmitterX->sampleBufferIndex - mobileAverageSampleIndex;
      if (mobilePointer <= 0) {
        mobilePointer = mobilePointer + (transmitterX->sampleBufferSize -1);
      }
      transmitterX->mobileAverageTemp = transmitterX->mobileAverageTemp + transmitterX->sampleBuffer[mobilePointer];
    }
    transmitterX->mobileAverage = transmitterX->mobileAverageTemp / transmitterX->mobileAverageFilterSize;
    // filling in the mobile average buffer with the fresh new value
    transmitterX->mobileAverageBuffer[transmitterX->mobileAverageBufferIndex] = transmitterX->mobileAverage;  // to be fair, this buffer is filled but still ...really unused.
    // truth being said, I'm filling the transmitterX->mobileAverageBuffer for future logging purposes. (TBD)
    
    // since we have the current mobile average data, we can also extract the current variance data. 
    // the variable named "variance" at this point still contains the *previous* value of the variance   
    transmitterX->variancePrev = transmitterX->variance;
    // deviation of the current sample
    transmitterX->varianceSample = (sample - transmitterX->mobileAverageBuffer[transmitterX->mobileAverageBufferIndex])*(sample - transmitterX->mobileAverageBuffer[transmitterX->mobileAverageBufferIndex]);
    
    // FIRsecondOrderFilter operations // please note, I'm improperly using the term FIR here: there is an IIR component too. 
    transmitterX->FIRvarianceAvg = 0;
    if (accessPoints.secondOrderFilter >= 1) {
      // computing FIRvarianceAvg
      int FIRvarianceAvgTemp = 0;
      for (int varianceSampleIndex = 0; varianceSampleIndex < transmitterX->varianceBufferSize; varianceSampleIndex++) {
        FIRvarianceAvgTemp = FIRvarianceAvgTemp + transmitterX->varianceBuffer[varianceSampleIndex];
      }
      transmitterX->FIRvarianceAvg = (FIRvarianceAvgTemp / transmitterX->varianceBufferSize) / accessPoints.secondOrderAttenutationCoefficient;
      transmitterX->varianceSample = abs(transmitterX->varianceSample - transmitterX->FIRvarianceAvg); // subtracting the mobile average variance from the variance sample

     
    }
    
    
    // filling in the variance buffer
    transmitterX->varianceBuffer[transmitterX->varianceBufferIndex] = transmitterX->varianceSample;

    
    
    // the following is a mobile integrator filter that parses the circular buffer called varianceBuffer
    transmitterX->varianceIntegral = 0;
    int variancePointer = 0;
    for (int varianceBufferIndexTemp = 0; varianceBufferIndexTemp < transmitterX->varianceIntegratorLimit; varianceBufferIndexTemp++) {
     variancePointer = transmitterX->varianceBufferIndex - varianceBufferIndexTemp;
     if (variancePointer <=0) {
        variancePointer = variancePointer + (transmitterX->varianceBufferSize -1);
     }
     transmitterX->varianceIntegral = transmitterX->varianceIntegral + transmitterX->varianceBuffer[variancePointer]; // the full effect of this operation is to make the system more sensitive to continued variations of the RSSI, possibly meaning there's a moving object around the area.
    }
    // increasing and checking the variance buffer index
    transmitterX->varianceBufferIndex++;
    if ( transmitterX->varianceBufferIndex >= transmitterX->varianceBufferSize ) { // circular buffer, rewinding the index, if the buffer has been filled at least once, then we may start processing valid data
      transmitterX->varianceBufferIndex = 0;
      transmitterX->varianceBufferValid = 1; //please note we DO NOT need to have a fully validated buffer to work with the current M.A. data
    }
    // applying the autoregressive part
    transmitterX->varianceAR = (transmitterX->varianceIntegral + transmitterX->varianceAR) / 2; // the effect of this filter is to "smooth" down the signal over time, so it's a simple IIR (infinite impulse response) low pass filter. It makes the system less sensitive to noisy signals, especially those with a variance of less than 1dBm.

      // diagnostics section
    if (debugRadarMsg >= 2) {
      Serial.println("");
      Serial.print("multistatic_interference_radar_process(): sampleBufferValid: yes: ");
      Serial.print(" rxRSSI: "); 
      Serial.print(transmitterX->latestReceivedSample);
      Serial.print(" prcsRSSI: "); 
      Serial.print(sample);
      Serial.print(", mobileAverage: "); 
      Serial.print(transmitterX->mobileAverage);
      Serial.print(", deviation: "); 
      Serial.print(sample - transmitterX->mobileAverage);
      Serial.print(", variance: "); 
      Serial.print(transmitterX->varianceSample);
      Serial.print(", varianceIntegral: "); 
      Serial.print(transmitterX->varianceIntegral);
      Serial.print(", varianceAR: "); 
      Serial.println(transmitterX->varianceAR);

    }
    
    // assigning the values according to the settings
    transmitterX->variance = transmitterX->varianceSample; 
    
    if (transmitterX->enableAutoRegressive) {
      transmitterX->variance = transmitterX->varianceAR;
    }
    if (! transmitterX->enableAutoRegressive) {
      transmitterX->variance = transmitterX->varianceIntegral;
    }
    
    // note: we needed to point to the current mobile average data for future operations, so we increase the MA buffer index only as the last step
    transmitterX->mobileAverageBufferIndex++;
    if ( transmitterX->mobileAverageBufferIndex >= transmitterX->mobileAverageBufferSize ) { // circular buffer, rewinding the index, if the buffer has been filled at least once, then we may start processing valid data
      transmitterX->mobileAverageBufferIndex = 0;
      transmitterX->mobileAverageBufferValid = 1; //please note we DO NOT need to have a fully validated buffer to work with the current M.A. data
    }
    
  }

  
  // final check to determine if the detected variance signal is above the detection threshold, this is only done if enableThreshold > 0 
  if ((transmitterX->variance >= transmitterX->varianceThreshold) && (transmitterX->enableThreshold > 0)) {
    transmitterX->detectionLevel = transmitterX->variance;
    if (debugRadarMsg >= 1) {
    	Serial.print("multistatic_interference_radar_process(): detected variance signal above threshold: ");
    	Serial.print(transmitterX->detectionLevel);
    }
    return transmitterX->detectionLevel;
  }
  // variance signal under threshold, but otherwise valid?
  if ((transmitterX->variance < transmitterX->varianceThreshold) && (transmitterX->variance >= 0) && (transmitterX->enableThreshold > 0) ) {
    transmitterX->detectionLevel = 0;
    if (debugRadarMsg >= 2) {
    	Serial.print("multistatic_interference_radar_process(): variance signal under threshold: ");
    	Serial.print(transmitterX->variance);
    }
    return transmitterX->detectionLevel;
  }
  
  return transmitterX->variance; // if the sample buffer is still invalid at startup, an invalid value is returned: -1, else the raw variance signal is returned

}




uint8_t strongestBSSID[6] = {0};
int strongestRSSI = -100;
int strongestChannel = 0;
int strongestAPfound = 0; // if found set to 1


uint8_t * currentBSSID;
int currentRSSI = 0;
int currentChannel = 0;


char * multistatic_find_next_strongest_AP() {
  
}


int searchSlotByBSSID(uint8_t * searchBSSID) {  // returns -1 if not found, returns the slot index for the transmitters list if found
  int res = -1;
  int bssidScanOK = 0;

  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {
    bssidScanOK = 0;
    for (int bssidIndex = 0; bssidIndex < 6; bssidIndex++) { // compare with the recorded BSSID, byte by byte
        if (searchBSSID[bssidIndex] == accessPoints.BSSIDs[slotIndex][bssidIndex]) {
          bssidScanOK = 1; 
        } else {
          bssidScanOK = 0; // match failed, resetting the flag 
          break; // no matter wich single byte failed to match, we break out of the compare loop
        }
    } // BSSID compare loop
    if (bssidScanOK == 1) { // result found!
      return slotIndex;
    }
  } // main for loop

  // if we get up to this point, clearly no matching BSSID was found
  return res;
  
}

void checkTXlist() {
  // now checking the transmitters list
    for (int dbgSlotIndex = 0; dbgSlotIndex < accessPoints.transmittersListLen; dbgSlotIndex++) {
      Serial.print("checkTXlist(): debug parsing the extant transmitters list: accessPoints.APslotStatus[dbgSlotIndex] values: ");
      Serial.print(accessPoints.APslotStatus[dbgSlotIndex]);
      Serial.print(" and BSSID: ");
      for (int dbgbssidIndex = 0; dbgbssidIndex < 6; dbgbssidIndex++) {
        Serial.print(accessPoints.BSSIDs[dbgSlotIndex][dbgbssidIndex], HEX);
      }
      Serial.println("");
    }
}

int checkInvalidTXdata() {
  int res = 0;
  
  for (int itxSlotIndex = 0; itxSlotIndex < accessPoints.transmittersListLen; itxSlotIndex++) {
    if (accessPoints.APslotStatus[itxSlotIndex] == AP_SLOT_STATUS_VALID) {
      if ((accessPoints.BSSIDs[itxSlotIndex][0] == 0) && (accessPoints.BSSIDs[itxSlotIndex][1] == 0) && (accessPoints.BSSIDs[itxSlotIndex][2] == 0) && (accessPoints.BSSIDs[itxSlotIndex][3] == 0) && (accessPoints.BSSIDs[itxSlotIndex][4] == 0) && (accessPoints.BSSIDs[itxSlotIndex][5] == 0) ) {
        accessPoints.APslotStatus[itxSlotIndex] = AP_SLOT_STATUS_FREE;
        //accessPoints.transmittersData[itxSlotIndex].sampleBufferValid = 0;
        //accessPoints.transmittersData[itxSlotIndex].mobileAverageBufferValid = 0;
        //accessPoints.transmittersData[itxSlotIndex].varianceBufferValid = 0;
        accessPoints.transmittersData[itxSlotIndex].resetRequest = 1; // forces all of the above commented, to be done internally
        res++;
      }
    }
  }
  /*
  if (debugRadarMsg >= 17) {
        Serial.print("checkInvalidTXdata(): final check: calling checkTXlist(): ");
        checkTXlist();
  }
  */

  
  return res;
}


int checkInvalidRSSI() {  // returns how many invalid results have been found (how many transmitters that were previously OK, now have weak signals and are unsuitable. 

  int res = 0; // all fine
  
  // first, parse all the APs, detect if they are already in the data structures and if so, check the RSSI for adequate levels, if inadequate mark the slot for reinitialization

    // please note the BSSD and SSDI can be transferred by memcpy, for example: memcpy ( strongestBSSID, currentBSSID, (sizeof(uint8_t) * 6));

  // also note: accessPoints.discoveredNetworks has already been checked and guaranteed to have at least one result

  for (int netItem = 0; netItem < accessPoints.discoveredNetworks; netItem++) {
      currentBSSID = WiFi.BSSID(netItem);
      currentRSSI = WiFi.RSSI(netItem);
      currentChannel = WiFi.channel(netItem);
      if (debugRadarMsg >= 4) {
        Serial.print("multistatic_interference_radar(): processing netItem number: ");
        Serial.print(netItem);
        Serial.print(" with SSID: ");
        Serial.print(WiFi.SSID(netItem));
        Serial.print(" on channel: ");
        Serial.print(currentChannel);
        Serial.print(" with RSSI: ");
        Serial.println(currentRSSI);
        
      }
      /*
      if (debugRadarMsg >= 17) {
        Serial.print("checkInvalidRSSI(): calling checkTXlist(): ");
        checkTXlist();
      }
      */

      if ((currentRSSI > accessPoints.absoluteRSSIlimit) || (currentRSSI < 0)) { // we at least have a valid result // please note: this does not mean the RSSI isn't excessively low anyways
        // see if we already have it in a slot, also detect excessively low RSSI, in such instance empty the slots
        accessPoints.currentTransmitterIndex = searchSlotByBSSID(currentBSSID);
        if (accessPoints.currentTransmitterIndex >= 0) { // valid index, AP found
          if (currentRSSI < accessPoints.transmittersData[accessPoints.currentTransmitterIndex].minimum_RSSI) { // INVALID RSSI, let's clean the spot and mark it free
            //accessPoints.initComplete = 0; // I don't think we'll ever need this one
            accessPoints.transmittersData[accessPoints.currentTransmitterIndex].initComplete = 0;
            accessPoints.transmittersData[accessPoints.currentTransmitterIndex].sampleBufferValid = 0;
            accessPoints.transmittersData[accessPoints.currentTransmitterIndex].varianceBufferValid = 0;
            accessPoints.transmittersData[accessPoints.currentTransmitterIndex].mobileAverageBufferValid = 0;
            accessPoints.BSSIDs[accessPoints.currentTransmitterIndex][0] = 0; // YES, I know I could use memset. This looks ugly? Yes. Do I mind? Nope ;)
            accessPoints.BSSIDs[accessPoints.currentTransmitterIndex][1] = 0;
            accessPoints.BSSIDs[accessPoints.currentTransmitterIndex][2] = 0;
            accessPoints.BSSIDs[accessPoints.currentTransmitterIndex][3] = 0;
            accessPoints.BSSIDs[accessPoints.currentTransmitterIndex][4] = 0;
            accessPoints.BSSIDs[accessPoints.currentTransmitterIndex][5] = 0;
          
            accessPoints.APslotStatus[accessPoints.currentTransmitterIndex] = AP_SLOT_STATUS_INVALID; 

            //accessPoints.transmittersData[accessPoints.currentTransmitterIndex].sampleBufferValid = 0;
            //accessPoints.transmittersData[accessPoints.currentTransmitterIndex].mobileAverageBufferValid = 0;
            //accessPoints.transmittersData[accessPoints.currentTransmitterIndex].varianceBufferValid = 0;

            res++; // found at least one invalid result, let's increase the invalid results counter
            
            if (debugRadarMsg >= 3) {
              Serial.print("multistatic_interference_radar(): processing recorded AP: RSSI too low for netItem: ");
              Serial.print(WiFi.SSID(netItem));
              Serial.print(" on channel: ");
              Serial.print(currentChannel);
              Serial.print(" with RSSI: ");
              Serial.println(currentRSSI);
            } // debug messages end
          }
        }
        
      } // valid result detected, processing it
      
  } // main for cycle, scanning each result

  

  return res;

}


int searchScanResultsByBSSID(uint8_t * BSSIDtoSearch) { // receives a pointer to the BSSID 6 bytes array //returns -1 if the BSSID is not in the scan results, if found, it returns the scan index (netItem)
  int res = -1;
  int bssidScanOK = 0;

  for (int netItem = 0; netItem < accessPoints.discoveredNetworks; netItem++) {
      currentBSSID = WiFi.BSSID(netItem);
      //currentRSSI = WiFi.RSSI(netItem);
      //currentChannel = WiFi.channel(netItem);

      for (int bssidIndex = 0; bssidIndex < 6; bssidIndex++) { // compare with the recorded BSSID, byte by byte
        if (currentBSSID[bssidIndex] == BSSIDtoSearch[bssidIndex]) {
          bssidScanOK = 1; 
        } else {
          bssidScanOK = 0; // match failed, resetting the flag 
          break; // no matter wich single byte failed to match, we break out of the compare loop
        }
    } // BSSID compare loop
    if (bssidScanOK == 1) { // result found!
      return netItem; // we also show *where* it has been found
    }

  }

  // if we get up to this point, clearly no matching BSSID was found
  return res;
  
}

void serialPrintBSSID(uint8_t * localBSSID) {
  for (int bssidIndex = 0; bssidIndex < 6; bssidIndex++) {
    if (localBSSID == NULL) {
      Serial.print("NULL");
      return;
    }
    
    Serial.print(localBSSID[bssidIndex], HEX);
  }
}

int searchScanResultsByBSSIDslotIndex(int txSlotIndex) { // receives the index pointing to a specific position of the BSSID in the access points data structure //returns -1 if the BSSID is not in the scan results, if found it returns the scan index (netItem); if the slot status is not valid, return the slot index as there is no need to further clean it. 
  int res = -1;
  int bssidScanOK = 0;

  if ((accessPoints.APslotStatus[txSlotIndex] != AP_SLOT_STATUS_VALID) || ((accessPoints.BSSIDs[txSlotIndex][0] == 0) && (accessPoints.BSSIDs[txSlotIndex][1] == 0) && (accessPoints.BSSIDs[txSlotIndex][2] == 0) && (accessPoints.BSSIDs[txSlotIndex][3] == 0) && (accessPoints.BSSIDs[txSlotIndex][4] == 0) && (accessPoints.BSSIDs[txSlotIndex][5] == 0) )) {
    if (debugRadarMsg >= 5) {
      Serial.print("searchScanResultsByBSSIDslotIndex(): found slot already empty or clean or invalid: slot: ");
      Serial.println(txSlotIndex);
    }
    return txSlotIndex;
  }

  for (int netItem = 0; netItem < accessPoints.discoveredNetworks; netItem++) {
      currentBSSID = WiFi.BSSID(netItem);
      //currentRSSI = WiFi.RSSI(netItem);
      //currentChannel = WiFi.channel(netItem);
      if (debugRadarMsg >= 16) {
        Serial.print(" searchScanResultsByBSSIDslotIndex(): looking for stored BSSID: ");
        serialPrintBSSID(accessPoints.BSSIDs[txSlotIndex]);
        Serial.print(" vs netItem BSSID: ");
        serialPrintBSSID(currentBSSID);
      }

      for (int bssidIndex = 0; bssidIndex < 6; bssidIndex++) { // compare with the recorded BSSID, byte by byte
        /*
        if (debugRadarMsg >= 17) {
          Serial.print(" checking byte by byte: ");
          Serial.print(accessPoints.BSSIDs[txSlotIndex][bssidIndex], HEX);
          Serial.print(" vs: ");
          Serial.print(currentBSSID[bssidIndex], HEX);
        }
        */
        if (currentBSSID[bssidIndex] == accessPoints.BSSIDs[txSlotIndex][bssidIndex]) {
          bssidScanOK = 1; 
        } else {
          bssidScanOK = 0; // match failed, resetting the flag 
          break; // no matter wich single byte failed to match, we break out of the compare loop
        }
    } // BSSID compare loop
    if (bssidScanOK == 1) { // result found!
      if (debugRadarMsg >= 6) {
        Serial.print("searchScanResultsByBSSIDslotIndex(): matched BSSID for slot index: ");
        Serial.println(txSlotIndex);
      }
      return netItem; // we also show *where* it has been found
    }

  }

  if (debugRadarMsg >= 6) {
    Serial.print("searchScanResultsByBSSIDslotIndex(): will need to clean tx slot: NO matched BSSID for slot index: ");
    Serial.println(txSlotIndex);
  }

  // if we get up to this point, clearly no matching BSSID was found
  return res;
  
}



int checkDeadTransmitters() {
  int res = 0; // all fine
  int bssidSearchRes = 0;

  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {

    bssidSearchRes = searchScanResultsByBSSIDslotIndex(slotIndex);
    if ((bssidSearchRes < 0)|| ((accessPoints.BSSIDs[slotIndex][0] == 0) && (accessPoints.BSSIDs[slotIndex][1] == 0) && (accessPoints.BSSIDs[slotIndex][2] == 0) && (accessPoints.BSSIDs[slotIndex][3] == 0) && (accessPoints.BSSIDs[slotIndex][4] == 0) && (accessPoints.BSSIDs[slotIndex][5] == 0) )) { // on no AP found via the scan
      
      // clean the slot
      //accessPoints.initComplete = 0; // I don't think we'll ever need this one
      accessPoints.transmittersData[slotIndex].initComplete = 0;
      accessPoints.transmittersData[slotIndex].sampleBufferValid = 0;
      accessPoints.transmittersData[slotIndex].varianceBufferValid = 0;
      accessPoints.transmittersData[slotIndex].mobileAverageBufferValid = 0;
      accessPoints.BSSIDs[slotIndex][0] = 0; // YES, I know I could use memset. This looks ugly? Yes. Do I mind? Nope ;)
      accessPoints.BSSIDs[slotIndex][1] = 0;
      accessPoints.BSSIDs[slotIndex][2] = 0;
      accessPoints.BSSIDs[slotIndex][3] = 0;
      accessPoints.BSSIDs[slotIndex][4] = 0;
      accessPoints.BSSIDs[slotIndex][5] = 0;


      //accessPoints.transmittersData[slotIndex].sampleBufferValid = 0;
      //accessPoints.transmittersData[slotIndex].mobileAverageBufferValid = 0;
      //accessPoints.transmittersData[slotIndex].varianceBufferValid = 0;
      accessPoints.transmittersData[slotIndex].resetRequest = 1; // forces all of the above commented, to be done internally
          
      accessPoints.APslotStatus[slotIndex] = AP_SLOT_STATUS_INVALID; // transmitter disappeared / out of range

      accessPoints.initComplete = 0; // guess we need to reinit the AP list

      res++;
    }
    
  } // main for cycle, parse the transmitters data

  return res; // returns how many slots have been cleared from dead transmitters
  
}




int checkTransmitterArray() {  // if there are empty, free, invalid slots then the result is 0, if the slot array is full and ok, then the result is 1

  int res = 1; // all fine

  /*
  if (debugRadarMsg >= 17) {
    checkTXlist();
  }
  */
  
  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {
    
    if ((accessPoints.APslotStatus[slotIndex] == AP_SLOT_STATUS_FREE) || (accessPoints.APslotStatus[slotIndex] == AP_SLOT_STATUS_INVALID)|| (accessPoints.APslotStatus[slotIndex] == AP_SLOT_STATUS_INIT)) {
      res = 0;
    }
  }

  return res;

}


int searchScanResultInPowerRank(int scanRes) { // returns the ranking position if an item was already recorded, or -1 if not found

  int res = -1; // not found!
  int tempRes = -1;

  for (int scanItem = 0; scanItem < accessPoints.discoveredNetworks; scanItem++) {
    /*
    if (debugRadarMsg >= 19) {
      Serial.print("searchScanResultInPowerRank(): scanItem: ");
      Serial.print(scanItem);
      Serial.print(" looking for netItem: ");
      Serial.print(scanRes);
      Serial.print(" in accessPoints.scanIndexByPower[scanItem]: found: ");
      Serial.println(accessPoints.scanIndexByPower[scanItem]);
    }
    */
    tempRes = accessPoints.scanIndexByPower[scanItem];
    if (scanRes == tempRes) {
      res = scanItem;
    }
  }
  return res;
}

int localDCSRNIStrongestRSSI = ABSOLUTE_RSSI_LIMIT; // let's start from the smallest possible level.
int localDCSRNIStrongestResult = -1;

int detectCurrentStrongestRSSInetItem() {

  //uint8_t localCurrentBSSID[6] = {0};
  int localCurrentRSSI = ABSOLUTE_RSSI_LIMIT;
  //int localStrongestRSSI = ABSOLUTE_RSSI_LIMIT; // let's start from the smallest possible level. // moved outside to the global space and renamed
  //int localStrongestResult = -1; // moved outside to the global space and renamed
  int localScanItemFound = -1;
  int res = -1;

  localDCSRNIStrongestRSSI = ABSOLUTE_RSSI_LIMIT; // let'see, the idea is to totally exclude existing results from the search
  
  // processing each scan result
    for (int netItem = 0; netItem < accessPoints.discoveredNetworks; netItem++) {
        //localCurrentBSSID = WiFi.BSSID(netItem);
        //localCurrentRSSI = WiFi.RSSI(netItem);
        localScanItemFound = searchScanResultInPowerRank(netItem);
        
        if (localScanItemFound < 0) { // item not found in the ranking array! Let's evaluate it
          /*
          if (debugRadarMsg >= 18) {
            Serial.print("detectCurrentStrongestRSSInetItem():  current response: ");
            Serial.println(res);
          }
          */
                    
          localCurrentRSSI = WiFi.RSSI(netItem); // moved here. we don't want to read RSSI from already-ranked results. 
          if (localCurrentRSSI >= localDCSRNIStrongestRSSI) {
            localDCSRNIStrongestRSSI = localCurrentRSSI;
            localDCSRNIStrongestResult = netItem;
            if (debugRadarMsg >= 18) {
              Serial.print(" UPDATING localStrongestResult: ");
              Serial.print(localDCSRNIStrongestResult);
              Serial.print(" localStrongestRSSI: ");
              Serial.println(localDCSRNIStrongestRSSI);
            }
            res = netItem; // this will be the returned value, the overall strongest with the exception of already registered results. 
            break; // without this line you'd eventually be returning duplicate values, overwriting the first one!!!!
          }
        }
    }
  /*
  if (debugRadarMsg >= 18) {
    Serial.print("detectCurrentStrongestRSSInetItem():  returning response: ");
    Serial.println(res);
  }
  */
  return res;


}



void sortScanResultsByRSSI() {
  
  //localDCSRNIStrongestRSSI = ABSOLUTE_RSSI_LIMIT; // let's start from the smallest possible level. // commented, not needed
  localDCSRNIStrongestResult = -1;

  int currentStrongestResult = -1;  
  
  

  int newStrongestNetItem = -1;

  int scanItemFound = -1;

  // reset the scanIndexByPower array
  //memset (accessPoints.scanIndexByPower, 0, ABSOLUTE_MAX_SCAN_RESULTS * sizeof(byte));  // commented out, I need to debug this matter, also, I'm no longer using uint8_t, now it's int.
  for (int scanItem = 0; scanItem < accessPoints.discoveredNetworks; scanItem++) {
    accessPoints.scanIndexByPower[scanItem] = -1;
  }
  accessPoints.scanIndexByPowerFirstFreeSpot = 0;
  

    // processing each scan result
    for (int netItem = 0; netItem < accessPoints.discoveredNetworks; netItem++) {
        // detect the strongest overall netItem, excluding those already ranked
        newStrongestNetItem = detectCurrentStrongestRSSInetItem();
        scanItemFound = searchScanResultInPowerRank(netItem);
        /*
        if (debugRadarMsg >= 17) {
          Serial.print("sortScanResultsByRSSI():  newStrongestNetItem: ");
          Serial.print(newStrongestNetItem);
          Serial.print(" scanItemFound: ");
          Serial.println(scanItemFound);
        }
        */
        if (scanItemFound < 0) {
          // save the scan item in a new position
          /*
          if (debugRadarMsg >= 17) {
            Serial.print("sortScanResultsByRSSI():  scanItemFound less than 0 -> not found: updating scanIndexByPower array at index: ");
            Serial.print(accessPoints.scanIndexByPowerFirstFreeSpot);
            Serial.print(" newStrongestNetItem: ");
            Serial.print(newStrongestNetItem);
            Serial.print(" and SSID: ");
            Serial.println(WiFi.SSID(newStrongestNetItem));
          }
          */
          accessPoints.scanIndexByPower[accessPoints.scanIndexByPowerFirstFreeSpot] = newStrongestNetItem;
          accessPoints.scanIndexByPowerFirstFreeSpot++;
        }
    }


  if (debugRadarMsg >= 17) {
    Serial.print("sortScanResultsByRSSI(): sorted scan data follows:");
    Serial.println();
    for (int dgbSortItem = 0; dgbSortItem < accessPoints.discoveredNetworks; dgbSortItem++) {
      Serial.print("sortN: ");
      Serial.print(dgbSortItem);
      Serial.print(" netItemN: ");
      Serial.print(accessPoints.scanIndexByPower[dgbSortItem]);
      Serial.print(" RSSI: ");
      Serial.print(WiFi.RSSI(accessPoints.scanIndexByPower[dgbSortItem]));
      Serial.print(" SSID: ");
      Serial.print(WiFi.SSID(accessPoints.scanIndexByPower[dgbSortItem]));
      Serial.println();
    }

    // now checking the transmitters list
    checkTXlist();
    
    
  }

  
  
}


int loadSlotByNetItemIndex(int localNetItem, int localSlotIndex) {  // returns the slot position if the operation went OK, -1 otherwise
  int res = -1;

  uint8_t *localCurrentBSSID = WiFi.BSSID(localNetItem); // size 6 is fixed and hardwired
  int localCurrentRSSI = WiFi.RSSI(localNetItem);
  int localCurrentChannel = WiFi.channel(localNetItem);
  //uint8_t *localCurrentSSID = NULL;

  /*
  if (debugRadarMsg >= 17) {

      Serial.print("loadSlotByNetItemIndex(): setting accessPoints.APslotStatus[localSlotIndex] to: ");
      Serial.print(AP_SLOT_STATUS_VALID);
      Serial.print("; for localSlotIndex: ");
      Serial.println(localSlotIndex);
  }
  */
  accessPoints.APslotStatus[localSlotIndex] = AP_SLOT_STATUS_VALID;
  accessPoints.netItemNumbers[localSlotIndex] = (uint8_t)(localNetItem & 0xff); // updates the new netItem number // NOTE: AT THE MOMENT THIS VALUE IS UPDATED DIRECTLY BEFORE CALLING THIS FUNCTION, WE ARE UPDATING IT AGAIN BECAUSE WE MIGHT BE USING THIS FUNCTION ELSEWHERE IN THE FUTURE
  //memcpy(accessPoints.BSSIDs[localSlotIndex], localCurrentBSSID, (sizeof(uint8_t) * 6));
  for (int bssidIndex = 0; bssidIndex < 6; bssidIndex++) {
    accessPoints.BSSIDs[localSlotIndex][bssidIndex] = localCurrentBSSID[bssidIndex];
  }
  strncpy(accessPoints.SSIDs[localSlotIndex], WiFi.SSID(localNetItem).c_str(), 34);

  accessPoints.transmittersData[localSlotIndex].resetRequest = 1; // when a new tx is loaded o reloaded, it is customary to request a reset of any previous instance
  /*
  if (debugRadarMsg >= 17) {
      Serial.print("loadSlotByNetItemIndex(): loaded BSSID: ");
      serialPrintBSSID(accessPoints.BSSIDs[localSlotIndex]);
      Serial.print("; from BSSID: ");
      serialPrintBSSID(localCurrentBSSID);
      Serial.print("; to slot: ");
      Serial.print(localSlotIndex);
      Serial.print("; slot status number: ");
      Serial.print(accessPoints.APslotStatus[localSlotIndex]);
      Serial.print("; also loaded SSID: ");
      Serial.println(accessPoints.SSIDs[localSlotIndex]);
    }
  */


  return res;
}


int loadScanResults() { // return value: how many result have been loaded in the transmitter list // -1 if error, 0 if no trasmitters added

  int localNetItem = 0;
  int localCurrentRSSI = ABSOLUTE_RSSI_LIMIT;
  
  uint8_t * localCurrentBSSID;
  int localCurrentChannel = 0;
  char localCurrentSSID[34];

  int internalRes = -1;

  int scanItemFound = -1;
  

  int loadedRes = 0;  // return this value (count how many result have been loaded in the transmitter list
  
  for (int scanItem = 0; scanItem < accessPoints.discoveredNetworks; scanItem++) {

    localCurrentBSSID = WiFi.BSSID(accessPoints.scanIndexByPower[scanItem]);
    localCurrentRSSI = WiFi.RSSI(accessPoints.scanIndexByPower[scanItem]);
    localCurrentChannel = WiFi.channel(accessPoints.scanIndexByPower[scanItem]);
    //WiFi.SSID(accessPoints.scanIndexByPower[scanItem]).toCharArray(localCurrentSSID, 34);
    strncpy(localCurrentSSID, WiFi.SSID(accessPoints.scanIndexByPower[scanItem]).c_str(), 34);
    
    //////strcpy(accessPoints.SSIDs[do not exceed MAX_ALLOWED_TRANSMITTERS_NUMBER], localCurrentSSID); // note: I leave this line because the code is going to be re-used. 
    if (debugRadarMsg >= 4) {
      //Serial.print("loadScanResults(): sorted by RSSI: detected BSSID: ");
      serialPrintBSSID(localCurrentBSSID);
      Serial.print(" detected RSSI: ");
      Serial.print(localCurrentRSSI);
      Serial.print("; item number: ");
      Serial.print(scanItem);
      Serial.print("; net number: ");
      Serial.print(accessPoints.scanIndexByPower[scanItem]);
      Serial.print("; SSID: ");
      Serial.println(localCurrentSSID);
    }
    /*
    if (debugRadarMsg >= 17) {
      Serial.print("loadScanResults(): calling searchSlotByBSSID(localCurrentBSSID) with BSSID: ");
      serialPrintBSSID(localCurrentBSSID);
      Serial.print(" and SSID: ");
      Serial.println(localCurrentSSID);
    }
    */
    scanItemFound = searchSlotByBSSID(localCurrentBSSID); // search the BSSID, already registered?  // if > reports the slotIndex

    if (scanItemFound >= 0) {
      /*
      if (debugRadarMsg >= 17) {
        Serial.print("loadScanResults(): item found with BSSID: ");
        serialPrintBSSID(localCurrentBSSID);
        Serial.print(" and returned slot value: ");
        Serial.println(scanItemFound);
      }
      */
      accessPoints.netItemNumbers[scanItemFound] = (uint8_t)(scanItem & 0xff); // updates the netItem number for existing transmitters in the list. 
    }

    if (scanItemFound < 0) { // if not, proceed to load it on the first free slot // this is the most important part, we are doing it in order of strongest RSSI
      /*
      if (debugRadarMsg >= 17) {
        Serial.print("loadScanResults(): item NOT found for BSSID: ");
        serialPrintBSSID(localCurrentBSSID);
        Serial.print(" and returned slot value: ");
        Serial.println(scanItemFound);
      }
      */
      for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {
        /*
        if (debugRadarMsg >= 17) {
            Serial.print("loadScanResults(): locating free spots: parsing slotIndex: ");
            Serial.print(slotIndex);
            Serial.print(" and accessPoints.APslotStatus[slotIndex]: ");
            Serial.println(accessPoints.APslotStatus[slotIndex]);
        }
        */
        if ((accessPoints.APslotStatus[slotIndex] == AP_SLOT_STATUS_FREE) || (accessPoints.APslotStatus[slotIndex] == AP_SLOT_STATUS_INVALID) || (accessPoints.APslotStatus[slotIndex] == AP_SLOT_STATUS_INIT)) {
          /*
          if (debugRadarMsg >= 17) {
            Serial.print("loadScanResults(): detected available slot: proceeding to assign free slot: ");
            Serial.print(slotIndex);
            Serial.print(" and accessPoints.scanIndexByPower[scanItem]: ");
            Serial.println(accessPoints.scanIndexByPower[scanItem]);
          }
          */
          // The netItem number for the newly inserted transmitter is updated now
          accessPoints.netItemNumbers[slotIndex] = (uint8_t)(scanItem & 0xff); // WARNING: THIS NUMBER IS UPDATED AGAIN ALSO INSIDE THE loadSlotByNetItemIndex() FUNCTION, FOR RE-USABILITY REASONS
          
          internalRes = loadSlotByNetItemIndex(accessPoints.scanIndexByPower[scanItem], slotIndex); // loads the new transmitter data 
          
          if (internalRes >= 0) {
            loadedRes++;
          }
          break; // essential to prevent overwriting slots in case od duplicate RSSI levels.
        }
      }
    }
    

  }

  


  return loadedRes;
}



int multistatic_interference_radar_multiprocess() { // returns how many transmitters have been processed, or eventual error codes (values < 0).

  int res = 0; 
  int localCurrentNetItem = 0;
  int localCurrentRSSI = ABSOLUTE_RSSI_LIMIT;

  int totalVariance = 0; // this will be the returned value

  // reset the whole results array and other data arrays
  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {
    accessPoints.latestVariances[slotIndex] = 0;
  }
  // loads new results

  if (debugRadarMsg >= 1) { // formatting reasons
    Serial.print("multistatic_interference_radar(): ");
  }
  
  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {
    localCurrentNetItem = accessPoints.netItemNumbers[slotIndex];
    localCurrentRSSI = WiFi.RSSI(localCurrentNetItem);
    accessPoints.latestVariances[slotIndex] = multistatic_interference_radar_process(localCurrentRSSI, & accessPoints.transmittersData[slotIndex]); 
    //accessPoints.latestVariances[slotIndex] = multistatic_interference_radar_process(localCurrentRSSI, slotIndex);

    res++;
    totalVariance = totalVariance + accessPoints.latestVariances[slotIndex];
    
    // debugging info here
    if (debugRadarMsg >= 1) {
      Serial.print(" tx: ");
      Serial.print(slotIndex);
      Serial.print(" var: ");
      Serial.print(accessPoints.latestVariances[slotIndex]);
    } 

    // PROCESS ALARMS
    
    accessPoints.transmittersData[slotIndex].alarmStatus = 0; // first, clear the alarm
    if (accessPoints.transmittersData[slotIndex].enableThreshold >= 1) { // second, evaluate the threshold, if requested
      if (accessPoints.latestVariances[slotIndex] >= accessPoints.transmittersData[slotIndex].varianceThreshold) {
        accessPoints.transmittersData[slotIndex].alarmStatus = accessPoints.latestVariances[slotIndex]; // if triggered, update the alarm status with the variance value.
      }
    }

  } // main for cycle end
  
  if (debugRadarMsg >= 1) { // formatting reasons
    Serial.println();
  }

  //return res;
  return totalVariance;
  
}


void serialPrintScanResults() {

    Serial.print("serialPrintScanResults(): requested to print scan data:");
    Serial.println();
    for (int dgbSpNetItem = 0; dgbSpNetItem < accessPoints.discoveredNetworks; dgbSpNetItem++) {
      Serial.print("netItemN: ");
      Serial.print(dgbSpNetItem);
      Serial.print(" BSSID: ");
      serialPrintBSSID(WiFi.BSSID(dgbSpNetItem));
      Serial.print(" RSSI: ");
      Serial.print(WiFi.RSSI(dgbSpNetItem));
      Serial.print(" SSID: ");
      Serial.print(WiFi.SSID(dgbSpNetItem));
      Serial.print(" channel: ");
      Serial.print(WiFi.channel(dgbSpNetItem));
      
      Serial.println();
    }
}

void serialPrintCSVdata() {  // the Serial plotter function in Arduino IDE is smart enough to separately extract csv text data and use it as field names

  int localVariance = 0;
  int localTVariance = 0;

  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {

    Serial.print(accessPoints.SSIDs[slotIndex]);
    Serial.print("_RSSI");
    Serial.print(accessPoints.transmittersData[slotIndex].latestReceivedSample);
    Serial.print("_AVG");
    Serial.print(accessPoints.transmittersData[slotIndex].mobileAverage);
    Serial.print(",");
    
  }
  Serial.println();

  for (int slotIndex = 0; slotIndex < accessPoints.transmittersListLen; slotIndex++) {

    localVariance = accessPoints.latestVariances[slotIndex];
    localTVariance = localTVariance + localVariance;
    
    Serial.print(localVariance);
    Serial.print(",");
    
  }
  //Serial.print(","); // also graph the total variance
  //Serial.println(localTVariance); // also graph the total variance
  Serial.println();
  
  return;
}


int multistatic_interference_radar() { // request the RSSI level internally, then process the signal and return the detection level in dBm

  int rssi = 0;
  int scanRes = 0;

  int res = 0;


/* //// this part as a reference in case we decide to implement a more efficient scan type
  if (strongestAPfound == 0) { // don't have a strongest AP on record yet? Do a slow full scan
    scanRes = (int) WiFi.scanNetworks(false, false, false, 300, 0); //scanNetworks(bool async = false, bool show_hidden = false, bool passive = false, uint32_t max_ms_per_chan = 300, uint8_t channel = 0);
  } else { // do a single channel active scan, this should be much faster
    scanRes = (int) WiFi.scanNetworks(false, false, false, 200, strongestChannel); //scanNetworks(bool async = false, bool show_hidden = false, bool passive = false, uint32_t max_ms_per_chan = 300, uint8_t channel = 0);
  }
*/


  // even if it's slower, we always do a full channel scan. 

  accessPoints.discoveredNetworks = (int) WiFi.scanNetworks(false, false, false, 300, 0); //scanNetworks(bool async = false, bool show_hidden = false, bool passive = false, uint32_t max_ms_per_chan = 300, uint8_t channel = 0);  // channel 0 means all channels
  
  
  if (accessPoints.discoveredNetworks <= 0) {
    if (debugRadarMsg >= 1) {
      Serial.println("multistatic_interference_radar(): no connection or no AP in the vicinity: the radar is inoperable");
    }
    return RADAR_INOPERABLE;
  }

  // diagnostics

  if (debugRadarMsg >= 5) {
    serialPrintScanResults();
  }
  
  // safety checks on the results list

  if (accessPoints.discoveredNetworks >= ABSOLUTE_MAX_SCAN_RESULTS) {
    
    if (debugRadarMsg >= 1) {
      Serial.print("multistatic_interference_radar(): warning: detected an abnornally large number of transmitters during scan: ");
      Serial.print(accessPoints.discoveredNetworks);
      Serial.print("; please look into this matter as soon as possible: fixing this number to the maximum hardwired allowed limit: ");
      Serial.println(ABSOLUTE_MAX_SCAN_RESULTS);
    }
    accessPoints.discoveredNetworks = ABSOLUTE_MAX_SCAN_RESULTS;
  }

  
// now we'll do the reverse: parse the transmitters structure and clean transmitters that are no longer detected.

  
  res = checkInvalidTXdata(); // leave this, it is essential to correctly re-initialize the transmitters array

  if (debugRadarMsg >= 4) {
    Serial.print("checkInvalidTXdata(): cleaned invalid tx data slots: ");
    Serial.println(res);
  }

  res = checkDeadTransmitters();
  if (debugRadarMsg >= 4) {
    Serial.print("checkDeadTransmitters(): cleaned dead transmitters: ");
    Serial.println(res);
  }

  // invalid transmitter slots are marked and freed


// uncomment when debugging
  /* //// commenting this one, I'm moving the minimum_rssi logic inside the process function in order to implement a more elegant solution with the FIRsecondOrderFilter
  res = checkInvalidRSSI(); // parse the scan results and clean transmitters that were once valid but now have very low signals

  if ((debugRadarMsg >= 3) && (res > 0)) {
    Serial.print("multistatic_interference_radar(): transmitters lost to low signal that were once valid: ");
    Serial.println(res);
  }
  */

  // enable on debugging only
  if (accessPoints.RSSIcleanerEnable == 1) { // if enabled, parse the scan results and clean transmitters that were once valid but now have very low signals
    res = checkInvalidRSSI(); // leave this, it is essential to correctly re-initialize the transmitters array
    if ((debugRadarMsg >= 3) && (res > 0)) {
      Serial.print("multistatic_interference_radar(): transmitters lost to low signal that were once valid: ");
      Serial.println(res);
    }
  }


  // check the completeness of the data

  accessPoints.initComplete = checkTransmitterArray();  // if there are empty, free, invalid slots then the result is 0, if the slot array is full and ok, then the result is 1



  
  // fill empty slots if feasible, please note the BSSIDs must be unique occurrences in the array.
  
  if (accessPoints.initComplete == 0) { // need to initialize or reinitialize, as empty slots have been detected

    // sort the scan results by RSSI    // we do this part inside here on request since it's a bit computationally expensive.
    sortScanResultsByRSSI();

    

    // assign new slots if possible

    res = loadScanResults();
    if (debugRadarMsg >= 3) {
      Serial.print("multistatic_interference_radar(): loadScanResults() response: ");
      Serial.println(res);
    }
    
    

    // finalizing
    accessPoints.initComplete = 1;
  } // end filling empty slots

  

  
  if ((debugRadarMsg >= 3) && (res > 0)) {
    Serial.print("multistatic_interference_radar(): transmitters gone out of range or dead: ");
    Serial.println(res);
  }

  if (accessPoints.initComplete >= 1) { // process the data
    
    res = multistatic_interference_radar_multiprocess(); // the returned value is a cumulative measure of the signal's variance. Data relative to each transmitter is saved within the relative structures and can be accessed globally.

  }

  if (accessPoints.serialCSVdataEnable > 0) {
    serialPrintCSVdata();
  }


  //res = multistatic_interference_radar_multiprocess(); // the returned value is a cumulative measure of the signal's variance. Data relative to each transmitter is saved within the relative structures and can be accessed globally.

  return res;

}





int multistatic_interference_radar_debug_via_serial(int debugLevel) {

 int debugSave = debugRadarMsg;
 debugRadarMsg = debugLevel;
 if (debugRadarMsg >= 1) {
  Serial.print("multistatic_interference_radar_debug_via_serial(): debugging functions (if the current wifi mode allows it):");
  Serial.println(multistatic_interference_radar());
  
  /*
  int modeRes = (int) WiFi.getMode();
  
  if (modeRes & WIFI_MODE_NULL) {
    Serial.print("multistatic_interference_radar_debug_via_serial(): WIFI_MODE_NULL detected: can do nothing useful");
    return 0;
  }

  Serial.print("multistatic_interference_radar_esp() output:");
  Serial.println(multistatic_interference_radar());

  if ((modeRes & WIFI_MODE_APSTA) || (modeRes & WIFI_MODE_STA)) {
    Serial.print("multistatic_get_rssi_ScanStrongestAP() output:");
    Serial.println(bistatic_get_rssi_ScanStrongestAP());
  }
  if ((modeRes & WIFI_MODE_APSTA) || (modeRes & WIFI_MODE_AP)) {
    Serial.print("multistatic_get_rssi_SoftAP_strongestClient() output:");
    Serial.println(multistatic_get_rssi_SoftAP_strongestClient());
  }
  */
  
 }

 debugRadarMsg = debugSave; // restore the normal debug level
  
}


int multistatic_interference_radar_set_debug_level(int debugLevel) {
  debugRadarMsg = debugLevel;
}






int multistatic_interference_radar_set_txN_limit(int txNlimit) { // forcefully reduces the number of processed transmitters; CAVEAT: do not exceed MAX_ALLOWED_TRANSMITTERS_NUMBER
  
  if (debugRadarMsg >= 1) {
    Serial.print("multistatic_interference_radar_set_txN_limit(): requesto set accessPoints.transmittersListLen to: ");
    Serial.println(txNlimit);
  }
  
  if (txNlimit > MAX_ALLOWED_TRANSMITTERS_NUMBER) { // safety check, otherwise the array amd struct boundaries may be exceed. 
    txNlimit = MAX_ALLOWED_TRANSMITTERS_NUMBER;
  }
  accessPoints.transmittersListLen = txNlimit;
  if (debugRadarMsg >= 1) {
    Serial.print("multistatic_interference_radar_set_txN_limit(): set accessPoints.transmittersListLen to: ");
    Serial.println(accessPoints.transmittersListLen);
  }
  return txNlimit;

}


int multistatic_interference_radar_enable_second_order_variance_filtering(int FIRfilter) { // [ 0 = disabled, >=1 = enabled (1 recommended) ] enhances the second order derivative for very slow drifting variances, nullifies long-term variance offsets in crowded environments. As a wanted side-effect, when a transmitter accidentally becomes to weak to measure reliably and data stops being gathered, the terminal variance offset is gracefully compensated by the finite impulse response filter. 
  if (FIRfilter <0) {
    FIRfilter = 0;
  }
  accessPoints.secondOrderFilter = FIRfilter;
  return FIRfilter;
}


int multistatic_interference_radar_enable_aggressive_cleaning_low_RSSI(int cleanerEnable) {
  accessPoints.RSSIcleanerEnable = cleanerEnable;
  return accessPoints.RSSIcleanerEnable;
}


int multistatic_interference_radar_enable_serial_CSV_graph_data(int serialCSVen = 0) {

  accessPoints.serialCSVdataEnable = serialCSVen;
  debugRadarMsg = 0;
  return accessPoints.serialCSVdataEnable;
  
}



int multistatic_interference_radar_set_Second_Order_Attenutation_Coefficient(int attnCoeff) {
  accessPoints.secondOrderAttenutationCoefficient = attnCoeff;
  return attnCoeff;
}


int multistatic_interference_radar_set_minimum_RSSI(int rssiMin) {

  if ((rssiMin > 0) || (rssiMin < ABSOLUTE_RSSI_LIMIT)) {
    rssiMin == ABSOLUTE_RSSI_LIMIT; // which results in disabling the minimum RSSI check
  }

  

  for (int RSSIslotIndex = 0; RSSIslotIndex < accessPoints.transmittersListLen; RSSIslotIndex++) {
    if (debugRadarMsg >= 2) {
      Serial.print("multistatic_interference_radar_set_minimum_RSSI(): current accessPoints.transmittersData[N].minimum_RSSI: ");
      Serial.print(accessPoints.transmittersData[RSSIslotIndex].minimum_RSSI);
      Serial.print(" for slot ");
      Serial.print(RSSIslotIndex);
      Serial.print("/");
      Serial.println(accessPoints.transmittersListLen);
    }
    accessPoints.transmittersData[RSSIslotIndex].minimum_RSSI = rssiMin;
    if (debugRadarMsg >= 1) {
      Serial.print("multistatic_interference_radar_set_minimum_RSSI(): set accessPoints.transmittersData[N].minimum_RSSI to: ");
      Serial.println(accessPoints.transmittersData[RSSIslotIndex].minimum_RSSI);
    }
  }
  
  return rssiMin;
}


// current status: IMPLEMENTED
int multistatic_interference_radar_enable_alarm(int enableThreshold) {
  if (enableThreshold < 0) {
    enableThreshold = 0; // which results in disabling the alarm
  }

  

  for (int enableslotIndex = 0; enableslotIndex < accessPoints.transmittersListLen; enableslotIndex++) {
    
    accessPoints.transmittersData[enableslotIndex].enableThreshold = enableThreshold;
    if (debugRadarMsg >= 1) {
      Serial.print("multistatic_interference_radar_enable_alarm(): set accessPoints.transmittersData[N].enableThreshold to: ");
      Serial.println(accessPoints.transmittersData[enableslotIndex].enableThreshold);
    }
  }
  
  return enableThreshold;
  
}


// current status: IMPLEMENTED
int multistatic_interference_radar_set_alarm_threshold(int alarmThreshold) {
    if (alarmThreshold < 0) {
    alarmThreshold = 0; // which results in always enabling the alarm
  }

  

  for (int alarmslotIndex = 0; alarmslotIndex < accessPoints.transmittersListLen; alarmslotIndex++) {
    
    accessPoints.transmittersData[alarmslotIndex].varianceThreshold = alarmThreshold;
    if (debugRadarMsg >= 1) {
      Serial.print("multistatic_interference_radar_set_alarm_threshold(): set accessPoints.transmittersData[N].varianceThreshold to: ");
      Serial.println(accessPoints.transmittersData[alarmslotIndex].varianceThreshold);
    }
  }
  
  return alarmThreshold;
}

//
