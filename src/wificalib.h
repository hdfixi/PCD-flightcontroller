    #include <WiFi.h>

const char* ssid = "TOPNET_2810";
const char* password = "2kizac19mc";

WiFiServer server(80);

double Kp = 0.0;
double Kd = 0.0;
double Ki = 0.0;
void setupwificalib(){
      WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  Serial.println("Server started");
}
void mainwificalib(){
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
    else if (request.indexOf("/kd+") != -1) {
      Kd += 0.01;
    }
    else if (request.indexOf("/kd-") != -1) {
      Kd -= 0.01;
    }
    else if (request.indexOf("/ki+") != -1) {
      Ki += 0.01;
    }
    else if (request.indexOf("/ki-") != -1) {
      Ki -= 0.01;
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
    client.println("</p><a href=\"/kp+\"><button>+</button></a>");
    client.println("<a href=\"/kp-\"><button>-</button></a><br>");
    client.print("<h1>Kd:</h1><p>Number: ");
    client.print(Kd);
    client.println("</p><a href=\"/kd+\"><button>+</button></a>");
    client.println("<a href=\"/kd-\"><button>-</button></a><br>");
    client.print("<h1>Ki:</h1><p>Number: ");
    client.print(Ki);
    client.println("</p><a href=\"/ki+\"><button>+</button></a>");
    client.println("<a href=\"/ki-\"><button>-</button></a><br>");
    client.println("</body></html>");

    client.stop();
      Serial.print("kp:");
    Serial.print(Kp);
     Serial.print(" kd:");
    Serial.print(Kp);
     Serial.print(" ki:");
    Serial.print(Kp);
    Serial.println("Client disconnected");
  }
}