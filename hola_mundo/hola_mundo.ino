/**

UNIVERSIDAD RAFAEL LANDÍVAR
Facultad de Ingeniería
Curso: Arquitectura del Computador
  
Evidencia: Hola Mundo
 Integrantes:
- Carlos Suruy - 1213124
- Daniel Zepeda - 1265824
 
Descripción:
Programa inicial "Hola Mundo"
**/

void setup() {
  Serial.begin(9600);
}

void loop() {
  Serial.println("Hola Mundo");
  delay(1000);
}