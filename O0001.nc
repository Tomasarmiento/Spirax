%
O0001 (SPIRAX - DISPATCHER PRINCIPAL)
( ============================================================ )
( Programa principal que coordina con la PC via macros:        )
(   #500 = TIPO de pieza (1-6, 0 = sin pieza)                  )
(   #501 = OPERACION (10 o 20, 0 = sin pieza)                  )
(   #503 = FLAG fin de mecanizado (NC pone 1, PC limpia a 0)   )
(                                                              )
( La PC mantiene #500/#501 actualizadas con el primer elemento )
( de la cola. Cuando NC termina una pieza, setea #503=1 y      )
( espera a que la PC limpie a 0 antes de seguir.               )
( ============================================================ )

#503 = 0  (libre al arrancar)

N100 (LOOP PRINCIPAL)

(--- Esperar a que haya pieza valida en la cola ---)
IF [#500 EQ 0] GOTO 100
IF [#501 EQ 0] GOTO 100

(--- Snapshot local: copio macros antes de pickear ---)
( Esto evita que cambios de la PC durante el ciclo afecten )
( la pieza que ya esta siendo procesada. )
#100 = #500  (TIPO local)
#101 = #501  (OPERACION local)

(--- Esperar a que llegue pieza al stopper ---)
(WAIT FOR sensor de pieza en stopper - depende del torno)
(M-code o entrada digital que indique "hay pieza para pickear")

(--- Pickear con el gantry ---)
(M-code que activa el gantry para cargar la pieza)

(--- Ramificar segun TIPO y OPERACION ---)
IF [#100 EQ 1] GOTO 1000  (Tipo 1)
IF [#100 EQ 2] GOTO 2000  (Tipo 2)
IF [#100 EQ 3] GOTO 3000  (Tipo 3)
IF [#100 EQ 4] GOTO 4000  (Tipo 4)
IF [#100 EQ 5] GOTO 5000  (Tipo 5)
IF [#100 EQ 6] GOTO 6000  (Tipo 6)
GOTO 9000  (default: tipo no valido)

( ===== Tipo 1 ===== )
N1000
IF [#101 EQ 10] GOTO 1010  (OP10)
IF [#101 EQ 20] GOTO 1020  (OP20)
GOTO 9000

N1010 (TIPO 1 - OP10: primera operacion)
( aca va el codigo de mecanizado op10 de tipo 1 )
( G54 G0 X100 Z50 ... etc ... )
GOTO 9999

N1020 (TIPO 1 - OP20: segunda operacion)
( aca va el codigo de mecanizado op20 de tipo 1 )
GOTO 9999

( ===== Tipo 2 ===== )
N2000
IF [#101 EQ 10] GOTO 2010
IF [#101 EQ 20] GOTO 2020
GOTO 9000

N2010 (TIPO 2 - OP10)
( a implementar )
GOTO 9999

N2020 (TIPO 2 - OP20)
( a implementar )
GOTO 9999

( ===== Tipos 3-6: agregar bloques iguales ===== )
N3000
N4000
N5000
N6000
GOTO 9000

( ===== ERROR: tipo o op invalida ===== )
N9000
#3000 = 100 (TIPO/OP INVALIDOS EN MACROS)
( El M-code 3000 dispara alarma. PC ve la alarma. )
GOTO 9999

( ===== FIN DE MECANIZADO ===== )
N9999
(--- Soltar pieza al stopper de salida ---)
(M-code del gantry para descargar)

(--- Avisar a la PC que terminamos esta pieza ---)
#503 = 1

(--- Esperar a que la PC confirme (limpie #503 a 0) ---)
N9990
IF [#503 NE 0] GOTO 9990

(--- Volver a esperar nueva pieza ---)
GOTO 100

M30
%
