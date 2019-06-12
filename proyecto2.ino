#include <Fuzzy.h>
#include <FastPID.h>


//PID
float Kp = 13.088, Ki = 0.009, Kd = 0.0005, Hz = 4000;
int output_bits = 8;
bool output_signed = false;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

// FUZZY
//Declarar objeto Fuzzy
Fuzzy* fuzzy = new Fuzzy();

//Declarar entradas fuzzy como formas trapezoidales
FuzzySet* muybajo = new FuzzySet(0, 0, 10, 20);
FuzzySet* bajo = new FuzzySet(15, 20, 30, 50);
FuzzySet* fmedio = new FuzzySet(45, 50, 50, 60);
FuzzySet* alto = new FuzzySet(55, 55, 70, 80);
FuzzySet* muyalto = new FuzzySet(75, 80, 90, 120);

//Declarar salidas Fuzzy como formas trapezoidales
FuzzySet* parado = new FuzzySet(0, 1, 1, 2);
FuzzySet* lento = new FuzzySet(10, 20, 40, 60);
FuzzySet* medio = new FuzzySet(50, 180, 180, 170);
FuzzySet* rapido = new FuzzySet(160, 180, 200, 210);
FuzzySet* muyrapido = new FuzzySet(200, 220, 255, 255);


//Pines
int INPIN = A0;
int OUTPIN = 9;

float NIVEL = 0;

float sp = 50;
bool pid = false;

//leer entradas analogas
float ReadInput() {
	float tval = (analogRead(INPIN) / 10);
	tval = tval * 1.1 - 13;
	if (tval > 100){ tval = 100; }
	if (tval < 0) { tval = 0; }
	return tval;	
}

//Actualizar objeto PID segun nuevos kp, ki, kd, hz, etc...
void UpdatePID() {
	 FastPID tmyPID(Kp, Ki, Kd, Hz, output_bits, output_signed);
	 myPID = tmyPID;
}

//Inicializar objetos fuzzy con sus valores
void FuzzySetup() {

	//entrada
	FuzzyInput* flujo = new FuzzyInput(1);
	flujo->addFuzzySet(bajo);
	flujo->addFuzzySet(muybajo);
	flujo->addFuzzySet(fmedio);
	flujo->addFuzzySet(alto);
	flujo->addFuzzySet(muyalto);
	fuzzy->addFuzzyInput(flujo);

	//salida
	FuzzyOutput* velocidadMotor = new FuzzyOutput(1);
	velocidadMotor->addFuzzySet(parado);
	velocidadMotor->addFuzzySet(lento);
	velocidadMotor->addFuzzySet(medio);
	velocidadMotor->addFuzzySet(rapido);
	velocidadMotor->addFuzzySet(muyrapido);
	fuzzy->addFuzzyOutput(velocidadMotor);

	//reglas fuzzy
	FuzzyRuleAntecedent* flujomuybajo = new FuzzyRuleAntecedent();
	flujomuybajo->joinSingle(muybajo);
	FuzzyRuleConsequent* velocidadMaxima = new FuzzyRuleConsequent();
	velocidadMaxima->addOutput(muyrapido);
	FuzzyRule* regla0 = new FuzzyRule(1, flujomuybajo, velocidadMaxima);
	fuzzy->addFuzzyRule(regla0);

	FuzzyRuleAntecedent* flujoBajo = new FuzzyRuleAntecedent();
	flujoBajo->joinSingle(alto);
	FuzzyRuleConsequent* velocidadRapido = new FuzzyRuleConsequent();
	velocidadRapido->addOutput(rapido);
	FuzzyRule* regla1 = new FuzzyRule(1, flujoBajo, velocidadRapido);
	fuzzy->addFuzzyRule(regla1);

	FuzzyRuleAntecedent* flujomedio = new FuzzyRuleAntecedent();
	flujomedio->joinSingle(fmedio);
	FuzzyRuleConsequent* velocidadmedia = new FuzzyRuleConsequent;
	velocidadmedia->addOutput(medio);
	FuzzyRule* regla2 = new FuzzyRule(1, flujomedio, velocidadmedia);
	fuzzy->addFuzzyRule(regla2);

	FuzzyRuleAntecedent* flujoalto = new FuzzyRuleAntecedent();
	flujoalto->joinSingle(alto);
	FuzzyRuleConsequent* velocidadlento = new FuzzyRuleConsequent;
	velocidadlento->addOutput(lento);
	FuzzyRule* regla3 = new FuzzyRule(1, flujoalto, velocidadlento);
	fuzzy->addFuzzyRule(regla3);

	FuzzyRuleAntecedent* flujomuyalto = new FuzzyRuleAntecedent();
	flujomuyalto->joinSingle(muyalto);
	FuzzyRuleConsequent* velocidadparado = new FuzzyRuleConsequent;
	velocidadparado->addOutput(parado);
	FuzzyRule* regla4 = new FuzzyRule(1, flujomuyalto, velocidadparado);
	fuzzy->addFuzzyRule(regla4);
}

//Setup inicial
void setup() {
	pinMode(INPIN, INPUT);
	pinMode(OUTPIN, OUTPUT);
	Serial.begin(9600);
	FuzzySetup();
}

//Envía variables globales por serial además del dato de entrada
void SendData(int outputval) {
	Serial.print("AO = ");
	Serial.println(analogRead(INPIN));
	Serial.print("Nivel = ");
	Serial.println(NIVEL);
	Serial.print("PID = ");
	Serial.println(pid);
	Serial.print("Nivel de potencia = ");
	Serial.println(outputval/2.55);
	Serial.print("Set point = ");
	Serial.println(sp);
	Serial.print("PMW = ");
	Serial.println(outputval);
	Serial.print("KP = ");
	Serial.println(Kp, 4);
	Serial.print("KI = ");
	Serial.println(Ki, 4);
	Serial.print("KD = ");
	Serial.println(Kd,4);
}

//Ejecuta el control PID
void PIDControl(float& dist ,float& toutput){
	int feedback = dist;
	uint32_t before, after;
	before = micros();
	uint8_t output = myPID.step(sp, feedback);
	// output = 255 - output; //invertir
	after = micros();	
	toutput = (float)output;
}

//Ejecuta el control Fuzzy
void FuzzyControl(float& dist, float& toutput) {
	fuzzy->setInput(1, dist);
	fuzzy->fuzzify();
	toutput = fuzzy->defuzzify(1);
}

//Establece el SP según los datos recibidos por la interfaz serial luego de recibir un byte clave.
void SetSPSerial(float& sp) {
	uint8_t tlow = Serial.read();
	uint8_t thigh = Serial.read();
	int16_t ttval = thigh << 8 | (tlow & 0xFF);
	sp = (float)ttval;
}

//Lee las entradas serial (ASCII)
void ReadSerialInput(float& tsp){
	int tmsg = Serial.read();

	switch (tmsg)
	{
	case 49: //49 es Char 1
		tsp++;
		break;
	case 50: //50 es Char 2
		tsp = tsp - 1;
		break;
	case 51: //51 es Char 3
		pid = !pid;
		break;
	case 52: //52 es Char 4
		tsp = tsp - 10;
		break;
	case 53: //53 es Char 5
		tsp = tsp + 10;
		break;
	case 54: //54 es Char 6
		Kp = Kp + 0.01;
		UpdatePID();
		break;
	case 55: //55 es Char 7
		Kp = Kp - 0.01;
		UpdatePID();
		break;
	case 56: //56 es Char 8
		Ki = Ki + 0.001;
		UpdatePID();
		break;
	case 57: //57 es Char 9
		Ki = Ki - 0.001;
		UpdatePID();
		break;
	case 58: //58 es Char :
		Kd = Kd + 0.0001;
		UpdatePID();
		break;
	case 59: //59 es Char ;
		Kd = Kd - 0.0001;
		UpdatePID();
		break;
	case 60: //60 es Char <
		tsp = 0;
		break;
	case 61: //61 es Char =
		tsp = 1200;
		break;
	case 62: //62 es Char >		
		SetSPSerial(tsp);
		break;
	default:
		break;
	}
}

//Loop
void loop() {
	NIVEL = ReadInput();
	ReadSerialInput(sp);
	float spdiff = NIVEL + 40 - sp;
	float POWER = 0;

	if (pid) {
		PIDControl(spdiff, POWER);
	}
	else {
		FuzzyControl(spdiff, POWER);
	}
	analogWrite(OUTPIN, POWER);
	SendData(POWER);	

	//delay(10);  //reduce el consumo de CPU pero es perjudicial ya que afecta la velocidad de respuesta del sistema.
}



