PWM: Servo motor 
================

This is an app which control a servo motor using PWM.

Board
+++++

STM32 NUCLEO-L476RG

Wiring
++++++

Servo::

 GND (brown)		  - board GND
 5V (red)		  - board 5V
 Signal (orange)	  - A0 PIN

Zephyr Environment Variables Setup
++++++++++++++++++++++++++++++++++

.. code:: 

     cd ~/F1tenth/
     . ./scripts/zephyr_env_setup.sh

Building And Flashing Servo Application
+++++++++++++++++++++++++++++++++++++++

.. code:: 

     cd ~/F1tenth/servo/
     west build -p auto -b nucleo_l476rg .
     west flash
