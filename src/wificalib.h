    #include <WiFi.h>
 
   
// const char* ssid = "TOPNET_2810";
// const char* password = "2kizac19mc";
const char* ssid = "GalaxyA12";
const char* password = "iheb2024";
WiFiServer server(80);

float Kp = 0.00f;
float Kd = 0.00f;
float Ki = 0.0f;


void setupwificalib(){
//      if (!SPIFFS.begin()) {
//     Serial.println("Failed to mount file system");
//     return;
//   }
//   file = SPIFFS.open("/text.txt",FILE_WRITE);

 
      WiFi.begin(ssid, password);
      digitalWrite(16,0);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  digitalWrite(16,1);
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");
}



void mainwificalib(){
     
//   if (!file) {
//     Serial.println("Failed to open file for writing");
//     return;
//   }
    WiFiClient client = server.available();
  if (client) {
    Serial.println("New client connected");
    String request = client.readStringUntil('\r');
    Serial.println(request);
    client.flush();

    if (request.indexOf("/kp+") != -1) {
      Kp += 0.01;
    }
    else if (request.indexOf("/kp-") != -1) {
      Kp -= 0.01;
    }
   else if (request.indexOf("/kpm+") != -1) {
      Kp += 0.1;
    }
    else if (request.indexOf("/kpm-") != -1) {
      Kp -= 0.1;
    }
    else if (request.indexOf("/kpmm") != -1) {
      Kp += 1.;
    }
    else if (request.indexOf("/kpmn") != -1) {
      Kp -= 1.;
    }
      else if (request.indexOf("/kp5+") != -1) {
      Kp += 5.;
    }
    else if (request.indexOf("/kp5-") != -1) {
      Kp -= 5.;
    }
    else if (request.indexOf("/kd+") != -1) {
      Kd += 0.1;
    }
    else if (request.indexOf("/kd-") != -1) {
      Kd -= 0.1;
    }
      else if (request.indexOf("/kdm+") != -1) {
      Kd += 1.;
    }
    else if (request.indexOf("/kdm-") != -1) {
      Kd -= 1.;
    }
       else if (request.indexOf("/kd10+") != -1) {
      Kd += 10.;
    }
    else if (request.indexOf("/kd10-") != -1) {
      Kd -= 10.;
    }
        else if (request.indexOf("/kd5+") != -1) {
      Kd += 5.;
    }
    else if (request.indexOf("/kd5-") != -1) {
      Kd -= 5.;
    }
    else if (request.indexOf("/ki+") != -1) {
      Ki += 0.001;
    }
    else if (request.indexOf("/ki-") != -1) {
      Ki -= 0.001;
    }
      else if (request.indexOf("/ki0+") != -1) {
      Ki += 0.01;
    }
    else if (request.indexOf("/ki0-") != -1) {
      Ki -= 0.01;
    }
      else if (request.indexOf("/ki00+") != -1) {
      Ki += 0.1;
    }
    else if (request.indexOf("/ki00-") != -1) {
      Ki -= 0.1;
    }
     

    client.println("HTTP/1.1 200 OK");
    client.println("Content-Type: text/html");
    client.println();
    client.println("<!DOCTYPE html>");
    client.println("<html>");
    client.println("<head><title>ESP32 Increment by Button Example</title></head>");
    client.println("<body>");
    client.print("<h1>Kp:</h1><p>Number: ");
    client.print(Kp);
    client.println("</p><a href=\"/kp+\"><button>+0.01</button></a>");
    client.println("<a href=\"/kp-\"><button>-0.01</button></a><br>");
    client.println("</p><a href=\"/kpm+\"><button>+0.1</button></a>");
    client.println("<a href=\"/kpm-\"><button>-0.1</button></a><br>");
    client.println("</p><a href=\"/kpmm\"><button>+1</button></a>");
    client.println("<a href=\"/kpmn\"><button>-1</button></a><br>");
    client.println("</p><a href=\"/kp5+\"><button>+5</button></a>");
    client.println("<a href=\"/kp5-\"><button>-5</button></a><br>");
    client.print("<h1>Kd:</h1><p>Number: ");
    client.print(Kd);
    client.println("</p><a href=\"/kd+\"><button>+0.1</button></a>");
    client.println("<a href=\"/kd-\"><button>-0.1</button></a><br>");
    client.println("</p><a href=\"/kdm+\"><button>+1</button></a>");
    client.println("<a href=\"/kdm-\"><button>-1</button></a><br>");
     client.println("</p><a href=\"/kd10+\"><button>+10</button></a>");
    client.println("<a href=\"/kd10-\"><button>-10</button></a><br>");
     client.println("</p><a href=\"/kd5+\"><button>+5</button></a>");
    client.println("<a href=\"/kd5-\"><button>-5</button></a><br>");
    client.print("<h1>Ki:</h1><p>Number: ");
    client.print(Ki,3);
    client.println("</p><a href=\"/ki+\"><button>+0.001</button></a>");
    client.println("<a href=\"/ki-\"><button>-0.001</button></a><br>");
     client.println("</p><a href=\"/ki0+\"><button>+0.01</button></a>");
    client.println("<a href=\"/ki0-\"><button>-0.01</button></a><br>");
     client.println("</p><a href=\"/ki00+\"><button>+0.1</button></a>");
    client.println("<a href=\"/ki00-\"><button>-0.1</button></a><br>");
    client.println("</body></html>");
    
  
    
    client.stop();
    
    
    // Serial.print("kp:");
    // Serial.print(Kp);
    // Serial.print(" kd:");
    // Serial.print(Kd);
    // Serial.print(" ki:");
    // Serial.print(Ki);
    // Serial.println("Client disconnected");
  }
}
