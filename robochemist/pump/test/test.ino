/***************************************************
     DFRobot Gravity: Peristaltic Pump
     <https://www.dfrobot.com/wiki/index.php/Gravitry: Peristaltic Pump SKU:DFR0523>

     ***************************************************
     This sample code shows 3 states: clockwise maximum speed rotation; stop; counterclockwise maximum speed rotation

     Created 2017-12-25
     By Jason <jason.ling@dfrobot.com@dfrobot.com>

     GNU Lesser General Public License.
     See <http://www.gnu.org/licenses/> for details.
     All above must be included in any redistribution
     ****************************************************/

     /***********Notice and Trouble shooting***************
    0   -> clockwise maximum speed rotation
    90  -> stop
    180 -> counterclockwise maximum speed rotation
     ****************************************************/

    #include <Servo.h>

    Servo myservo;

    #define PUMPPIN 6    //peristaltic pump control pin, connect to arduino digital pin 9
    #define waitTime 4000 //interval time(ms) between every state

    void setup()
    {
      myservo.attach(PUMPPIN);
    }

    void loop()
    {
        myservo.write(0);   //Clockwise maximum speed rotation
        delay(waitTime);
        myservo.write(90);  //Stop
        delay(waitTime);
        myservo.write(180); //Counterclockwise maximum speed rotation
        delay(waitTime);
        myservo.write(90);  //Stop
        delay(waitTime);
    }
