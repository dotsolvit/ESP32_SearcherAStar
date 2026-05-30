//Funtion Pilot

//Pilot initialization
int pilotInit();

//Turn to the current point of the path
void pilotTurn();

//Pilot forward
int pilotForward(void) ;

//Pilot stop
void pilotStop(void) ;  

//The narrow scanner for detect a new obstacle
int pilotNarrowcanner(void);

//The circular scanner initiation 
void initCircularScanner();

//The circular scanner for detect a new obstacle
int pilotScannerCircular(void) ;

//Wide scan after stopping due to an obstacle
void pilotStopScanner(void) ;

//Journal:
void initJournal();
//Function to add a message to the journal
void addToJournal(const char* message);

//median filter: from three numbers, the program selects the one that is in the middle in value
int medianFilter(int a, int b, int c) ;