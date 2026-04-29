//funcWiFiWeb.h
//Модуль фунукццій для роботи з WiFi та веб-сервером на ESP32

//Connecting to WiFi in STA mode
void setupWiFi(void);

//Web
//Starting the web server
void Web_starting(void);

//Processing client requests
void handle_Client(void);

//Home page handler
void handle_OnConnect(void);

//Actions when pressing send go
void handle_sendgo(void);   

//Actions when pressing send y x
void handle_sendyx(void);

//Actions when pressing find path
void handle_findpath(void); 

//Function for generating an HTML page
String SendHTML(void);

//Actions when pressing See Map
void handle_seemap(void);
void handle_seemapup(void);
void handle_seemapdown(void);   

//Function for generating an HTML page Table
String SendSeeMapHTML(String mess);

//Формування рядка web_messageMap як карти перешкод та шляху
//Forming the web_messageMap string as a map of obstacles and path
String mapObstacleAndPath(int top_line_number);

//Знайти точку з координатами в одновимірній матриці координат
//Find a point with coordinates in a one-dimensional matrix of coordinates
int indexFindPointCoords( Coord *set, uint8_t setSize, Coord &c);
