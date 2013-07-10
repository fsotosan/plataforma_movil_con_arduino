

#define ESPERANDO_INICIO_TRAMA  0
#define ESPERANDO_VLINEAL_HI    1
#define ESPERANDO_VLINEAL_LO    2
#define ESPERANDO_VANGULAR_HI   3
#define ESPERANDO_VANGULAR_LO   4
#define ESPERANDO_FIN_TRAMA_1    5
#define ESPERANDO_FIN_TRAMA_2    6
#define TRAMA_OK                7

#define BYTE_INICIO_DE_TRAMA 'A'
#define BYTE_FIN_DE_TRAMA_1  'Z'
#define BYTE_FIN_DE_TRAMA_2  'Z'

#define MOTOR_MAX_RPM           230

/* Distancia entre ruedas y diametro rueda en mm */ 

#define LONGITUD_SEMIEJE_TRACCION_MM   80.0
#define DIAMETRO_RUEDA_MM              65.0

#define PIN_DIR_IZQUIERDA    12
#define PIN_DIR_DERECHA      13
#define PIN_FRENO_IZQUIERDA  9
#define PIN_FRENO_DERECHA    8
#define PIN_PWM_IZQUIERDA    3
#define PIN_PWM_DERECHA      11

unsigned char comando[8] = "";

unsigned char vLinealHi;
unsigned char vLinealLo;
unsigned char vAngularHi;
unsigned char vAngularLo;
signed int vLineal_mm_segundo;
signed int vAngular_grados_segundo;
unsigned char pwm_derecha;
unsigned char pwm_izquierda;
int estado;
float mm_segundo_a_rpm = 60 / (PI * DIAMETRO_RUEDA_MM);
float grados_segundo_a_rad_segundo = PI / 180.0;

int cuenta = 0;

void setup() {

  Serial.begin(9600);
  
  pinMode(PIN_DIR_DERECHA,HIGH);
  pinMode(PIN_DIR_IZQUIERDA,HIGH);
  pinMode(PIN_FRENO_DERECHA,LOW);
  pinMode(PIN_FRENO_IZQUIERDA,LOW);
  
  // Establecemos como salidas los pines correspondientes
  // a las señales de direccion y freno de los dos motores

  estado = ESPERANDO_INICIO_TRAMA;
  
  Serial.println("READY");
}

void printComando() {

  int i;
  for(i=0;i<7;i++) {
    Serial.print("comando[");
    Serial.print(i,DEC);
    Serial.print("] = ");
    if ((i==0)||(i==5)||(i==6)) {
      Serial.println(comando[i]);
    } else {
      Serial.print("0x");
      Serial.println(comando[i],HEX);
    }
  }
  
}

void loop() {
 
  unsigned char byteLeido;
  int i;
  
  if (Serial.available() > 0) {
    
    byteLeido=Serial.read();
    
    //Serial.println(byteLeido,DEC);
    
    if (byteLeido != 0) {
    
      switch(estado) {
      
        case ESPERANDO_INICIO_TRAMA:
        
          for(i=0;i<8;i++) {
             comando[i] = 0;
          }
          if (byteLeido==BYTE_INICIO_DE_TRAMA) {
             comando[estado++] = byteLeido;
          } else {
             //Serial.println("!A");
          }
          break;
          
        case ESPERANDO_VLINEAL_HI:
         
          comando[estado++] = byteLeido;
          break;
        
        case ESPERANDO_VLINEAL_LO:
        
          comando[estado++] = byteLeido;
          break;
         
        case ESPERANDO_VANGULAR_HI:
        
          comando[estado++] = byteLeido;
          break;
          
        case ESPERANDO_VANGULAR_LO:
        
          comando[estado++] = byteLeido;
          break;
          
        case ESPERANDO_FIN_TRAMA_1:
         
          comando[estado++] = byteLeido;
          if (byteLeido != BYTE_FIN_DE_TRAMA_1) {
            estado=ESPERANDO_INICIO_TRAMA;
            Serial.println("!Z1");
            printComando();
          }
        
          break;
         
        case ESPERANDO_FIN_TRAMA_2:
         
          comando[estado++] = byteLeido;
          if (byteLeido != BYTE_FIN_DE_TRAMA_2) {
            estado=ESPERANDO_INICIO_TRAMA;
            Serial.println("!Z2");
            printComando();
          }
        
          break;
          
        default:
        
          break;
  
        }
        
      } 
    
    }
    
    if (estado==TRAMA_OK) {
      
        float vLinealRuedaIzquierda_mm_segundo, vLinealRuedaDerecha_mm_segundo;
        float rpm_derecha, rpm_izquierda;
        float vComponenteLinealDeVelocidadAngular_mm_segundo;
        
        // Interpretamos comando
        
        for (i=1;i<=4;i++) {
          if(comando[i]=='0') {
            comando[i] = 0;
          }
        }
        
        vLinealHi = comando[1];
        vLinealLo = comando[2];
        vAngularHi = comando[3];
        vAngularLo = comando[4];
        
        // Recomponemos el dato de velocidad a partir de sus bytes alto y bajo
        
        vLineal_mm_segundo = ((unsigned int)vLinealHi)*256 + (unsigned int)vLinealLo;
        vAngular_grados_segundo = ((unsigned int)vAngularHi)*256 + (unsigned int)vAngularLo;
        
        Serial.print("*");
        Serial.print("Lin");
        Serial.print(vLineal_mm_segundo,DEC);
        Serial.print("Ang");
        Serial.print(vAngular_grados_segundo,DEC);
        Serial.print("*");
        
        // Consideramos vAngular positiva en sentido antihorario
        
        vComponenteLinealDeVelocidadAngular_mm_segundo = vAngular_grados_segundo * LONGITUD_SEMIEJE_TRACCION_MM * grados_segundo_a_rad_segundo;
        
        vLinealRuedaDerecha_mm_segundo = vLineal_mm_segundo + vComponenteLinealDeVelocidadAngular_mm_segundo; 
        vLinealRuedaIzquierda_mm_segundo = vLineal_mm_segundo - vComponenteLinealDeVelocidadAngular_mm_segundo;
        
        // Obtenemos las velocidades de giro de las ruedas en rpm
        
        rpm_derecha = vLinealRuedaDerecha_mm_segundo * mm_segundo_a_rpm;
        
        if (rpm_derecha > MOTOR_MAX_RPM) 
          rpm_derecha = MOTOR_MAX_RPM;
        else if (rpm_derecha < -MOTOR_MAX_RPM)
          rpm_derecha = -MOTOR_MAX_RPM;
          
        rpm_izquierda = vLinealRuedaIzquierda_mm_segundo * mm_segundo_a_rpm;
        
        if (rpm_izquierda > MOTOR_MAX_RPM) 
          rpm_izquierda = MOTOR_MAX_RPM;
        else if (rpm_izquierda < -MOTOR_MAX_RPM)
          rpm_izquierda = -MOTOR_MAX_RPM;
          
        // y las traducimos a duties PWM
        
        pwm_derecha = abs(rpm_derecha * 255 / MOTOR_MAX_RPM);
        pwm_izquierda = abs(rpm_izquierda * 255 / MOTOR_MAX_RPM);
        
        // Establecemos el sentido de giro
        
        digitalWrite(PIN_DIR_DERECHA,(rpm_derecha > 0)?LOW:HIGH);
        digitalWrite(PIN_DIR_IZQUIERDA,(rpm_izquierda > 0)?LOW:HIGH);
        
        // Enviamos duties PWM a motores
        
        analogWrite(PIN_PWM_IZQUIERDA,pwm_izquierda);
        analogWrite(PIN_PWM_DERECHA,pwm_derecha);
         
        // Preparamos la siguiente recepcion
        
        estado=ESPERANDO_INICIO_TRAMA;

    }

  }
   
  
