/* This is an Obstacle Avoiding Robot using Reinforcement Learning/AI
   Algorithm used in this project: Q learning

How Obstacle Avoiding Works?
1.Ultrasonic sensor is used measure distance from the obstacle using
  its Echo and Trig Pins.
2.When distance is measured and if its less than 20cm then there is
  an obstacle nearby otherwise robot is safe and continue Forward.
3.If obstacle is detected  then robot takes left or right turn depending
 on the situation.

How AI based Obstacle Avoidance Works?(Q learning)
1.Here the 1st step from upper article remains the same.However the
  2nd Step is different.
2.A list of actions of the robot are initialised first. For Example
 in this case actions of Robot are: Left , Forward, Backward ,Stop.
3.When the Robot comes near an obstacle it is required to perform an action.
  However note that in this case Robot doesn't know which action to take as its not pre
  programmed and going to learn on its own to avoid obstacles.
4.When Robot stops when there is an obstacle in front of it then it gets reward as 0
  When Robot stops and goes backward  it receives reward of -5
  When Robot continues to move forward ignoring the obstacles it receives reward of -10
  When Robot just moves left as soon as obstacle is detected it gets reward of +10
5.In this way Robot learns on its own to avoid obstacles by Reward Mechanism.*/

//////////ROBOT'S HARDWARE PARAMETERS////////////////////
int TRIG_PIN = 7;
int ECHO_PIN = 8;
int duration;
float distance;

int M1 = 13;
int M2 = 12;
int M3 = 11;
int M4 = 10;

bool Obstacle = false;
int FLAG;
/////////////////////////END/////////////////////////////

/////////////////////////////////////Q LEARNING PARAMETERS///////////////////////////////////////////
float ALPHA = 0.1;    //LEARNING RATE
float GAMMA = 0.5;    //DISCOUNT FACTOR
float EPSILON = 0.90; //EXPLORATION PARAMETER
int REWARD;           //REWARD FOR PERFORMING AN ACTION
int EPISODES  = 100;

int STATE;                        // CURRENT STATE OF THE ROBOT
int ACTION = 0;                   //ACTION PERFORMED BY THE ROBOT(0:FORWARD,1:BACKWARD ,2;STOP,3:LEFT)
float PROB;                       //USED FOR EPSILON DECAY 
bool ACTION_TAKEN = false;        //THIS VARIABLES TELLS US WHETHER AN ACTION IS TAKEN OR NOT
int NEXT_STATE;                   // NEXT STATE OF THE ROBOT
const int STATES = 10;            //NUMBER OF STATES IN ENVIRONMENT 
int ACTIONS[4] = {1,2,3,4};
const int NUMBER_OF_ACTIONS = 4; //TOTAL WE HAVE 4 ACTION FORWARD,BACKWARD,LEFT AND STOP


/*THIS IS THE Q MATRIX OR Q TABLE. THIS IS BASICALLY THE DIARY THAT ROBOT WILL LOOK INTO 
BEFORE PERFORMING AN ACTION.BASED ON THE ACTION THE ROBOT WILL EARN REWARD AND THE Q VALUE 
WILL BE UPDATED IN THIS Q TABLE. HERE I HAVE CONISDERED 10 STATES. I HAVE ASSUMED ALL STATES 
ARE DIFFERENT EVEN THOUGH THEY ARE SAME.BASICALLY OBSTACLE AVOIDING ROBOT CONTAINS ONLY TWO STATES
i.e: 
1:WHEN ITS AWAY FROM OBSTACLE 
2:WHEN ITS NEAR TO THE OBSTACLE
BUT HERE TO ILLUSTRATE MORE COMPLEX ENVIRONMENT I HAVE ASSUMED THERE ARE 10 DIFFERENT STATES HERE
EXPECTING SAME/DIFFERENT ACTION.*/

float Q[STATES][NUMBER_OF_ACTIONS] = {{0.0,0.0,0.0,0.0},  //MOST IMPORTANT OF ALL IS THE Q TABLE.
                                      {0.0,0.0,0.0,0.0},  //IT IS FORMED BY STATES AS ITS  ROWS 
                                      {0.0,0.0,0.0,0.0},  //AND COLLUMNS AS ITS NUMBER OF ACTIONS
                                      {0.0,0.0,0.0,0.0},  //INITIALISED TO ZERO IN THE START
                                      {0.0,0.0,0.0,0.0},  // THIS WILL UPDATED IN THE FUTURE.
                                      {0.0,0.0,0.0,0.0},
                                      {0.0,0.0,0.0,0.0},
                                      {0.0,0.0,0.0,0.0},
                                      {0.0,0.0,0.0,0.0},
                                      {0.0,0.0,0.0,0.0}};

/*THIS IS A REWARD MATRIX OR REWARD TABLE. THIS IS RESPONSIBLE FOR GIVING
REWARD TO ROBOT FOR PERFORMING PARTICULAR ACTION. IT STORES THE REWARD FOR 
EACH ACTION TAKEN AT STATE. THE REWARD WILL BE POSITIVE IF THE ACTION 
PERFORMED IS GOOD AND NEGATIVE IF ACTION YIELDS BAD RESULTS.*/

int REWARDS[STATES][NUMBER_OF_ACTIONS] = {{-10,-2,-1,10}, 
                                          {-10,-2,-1,10}, 
                                          {-10,-2,-1,10}, 
                                          {-10,-2,-1,10},  
                                          {-10,-2,-1,10},
                                          {-10,-2,-1,10},
                                          {-10,-2,-1,10},
                                          {-10,-2,-1,10},
                                          {-10,-2,-1,10},
                                          {-10,-2,-1,10}};            
////////////////////////////////////////////END///////////////////////////////////////////////////

////////////////Q LEARNING UPDATE PARAMETERS////////////
float Q_OLD;
float Q_NEW;
float Q_MAX;
//////////////////////////END//////////////////////////

void setup() 
{
  Serial.begin(9600);
  pinMode(TRIG_PIN,OUTPUT);
  pinMode(ECHO_PIN,INPUT);
  pinMode(M1,OUTPUT);
  pinMode(M2,OUTPUT);
  pinMode(M3,OUTPUT);
  pinMode(M4,OUTPUT);  
  randomSeed(analogRead(A0));
  STATE = 0;
  Serial.println("TRAINING WILL START IN 5 SECONDS:  ");
  delay(5000);
} 

////////////////////////////ROBOT'S FUNCTIONS/////////////////////////////////
void Forward()
{
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M3,LOW);
  digitalWrite(M4,HIGH); 
}

void Backward()
{
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M3,HIGH);
  digitalWrite(M4,LOW);
}

void Left()
{
  digitalWrite(M1,HIGH);
  digitalWrite(M2,LOW);
  digitalWrite(M3,LOW);
  digitalWrite(M4,HIGH);
}

void Right()
{
  digitalWrite(M1,LOW);
  digitalWrite(M2,HIGH);
  digitalWrite(M3,HIGH);
  digitalWrite(M4,LOW);
}

void Stop()
{
  digitalWrite(M1,LOW);
  digitalWrite(M2,LOW);
  digitalWrite(M3,LOW);
  digitalWrite(M4,LOW);
}

bool Obstacle_Avoider()
{
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  duration = pulseIn(ECHO_PIN ,HIGH);
  distance = (duration/2)/29.1;
  
  if(distance<15)
  { 
    Obstacle = true;
  }

  if(distance>15)
  {
    Obstacle = false;
  }
  
  delay(10);
  return Obstacle;
}
////////////////////////////////////////////END////////////////////////////////////////////////

///////////////////////////////ROBOT'S Q LEARNING FUNCTIONS////////////////////////////////////

float RANDOM(float EXPLORATION_PARAMETER)
{
  /*THIS FUNCTION FINDS RANDOM NUMBER WHICH
  DECIDES WHETHER AN ACTION TO BE TAKEN IS RANDOM
  OR FROM Q_TABLE*/
  
  float RANDOM_VARIABLE;
  float PROBABILITY;

  RANDOM_VARIABLE = random(0,100);
  PROBABILITY = RANDOM_VARIABLE/100;

  return PROBABILITY;
}


float DECAY(float PARAMETER)
{
  /*THIS FUNCTION IS USED TO REDUCE 
  EPSILON(EXPLORATION PARAMETER) WITH
  TIME.FINALLY AT THE END YOU GET RID 
  EPSILON AND THE ROBOT LEARNS TO AVOID
  OBSTACLES ON ITS OWN */
 
  PARAMETER = PARAMETER*0.98; //PARAMETER HERE IS THE EPSILON
  return PARAMETER;
}

int GET_STATE()
{
  int STATE_NUMBER;
  STATE_NUMBER = random(0,10);
  return STATE_NUMBER;
}


float MAX(float Q_Table[][4],int NEXT_S)
{ 
  /*THIS FUNCTION FINDS THE BIGGEST NUMBER
  IN Q_TABLE[NEXT_STATE]. THE MAIN ROLE OF
  THIS FUNCTION IS TO FIND Q_MAX PARAMETER*/
  
  float LIST[4];
  float N1;
  float N2;
  float MAX_VALUE= 0.0;
  float DIFF; 

  for(int b = 0; b<=3; b++)
     {
      LIST[b] = Q[NEXT_S][b];
     }

  for(int j = 0; j<=2 ; j++)
    {
      if(MAX_VALUE >LIST[j])
        {
          N1 = MAX_VALUE;
        }
      else
        {
         N1 = LIST[j];
        }
         
      N2 = LIST[j+1];
      DIFF = N1- N2;

      if(DIFF > 0)
        {
          MAX_VALUE = N1;
        }

      else
        {
          MAX_VALUE = N2;
        }
    }   
  return MAX_VALUE;
}


int ARGMAX(float Q_Table[][4],int S)
{
  /*THIS FUNCTION FINDS THE INDEX OF 
  BIGGEST Q VALUE IN Q TABLE[STATE]*/
  
   float ARRAY[4];
   float N1;
   float N2;
   float MAX_VALUE = 0.0;
   float DIFF;
   float NUMBER;
   int MAX_INDEX;

   for(int u= 0; u<=3; u++)
     {
       ARRAY[u] = Q_Table[S][u];
     }
   
   for(int p = 0; p<=2 ; p++)
    {
      if(MAX_VALUE >ARRAY[p])
        {
          N1 = MAX_VALUE;
        }
      else
        {
         N1 = ARRAY[p];
        }
         
      N2 = ARRAY[p+1];
      DIFF = N1- N2;

      if(DIFF > 0)
        {
          MAX_VALUE = N1;
        }

      else
        {  
          MAX_VALUE = N2;
        }
    }

  for(int r = 0; r<=3;r++)
     {
      NUMBER = ARRAY[r];
      if(NUMBER == MAX_VALUE)
        {
          MAX_INDEX  = r;
          break;
        }
     }
  
  return MAX_INDEX;
}

void Update(float Q_TABLE[][4] , int S, int NEXT_S, int A, int ACTIONS[], int R, float LEARNING_RATE, float DISCOUNT_FACTOR)
{
  /*THIS FUNCTION UPDATES THE Q TABLE AND Q VALUES. THIS UPDATE KEEPS ON HAPPENING UNTILL THE 
  MAIN LOOP ENDS. AT THE END OF EPISODES THE Q TABLE IS FILLED WITH VARIOUS VALUES. THE GREATER
  THE VALUES THE GREATER IMPORTANCE THE ACTION HAS AT THAT PARTICULAR STATE. "Q_OLD" IS OLD VALUE
  THAT THE Q MATRIX HAS.THIS IS THE VALUE WHICH GETS UPDATED EVENTUALLY. Q_NEW IS THE NEW Q_VALUE 
  WHICH IS CALCULATED BY THE Q LEARNING FORMULA. THE Q LEARNING FORMULA USED HERE IS BASED ON 
  BELLMAN EQUATION USES TEMPORAL DIFFERENCE LEARNING APPROACH.(MONTE CARLO APPROACH WILL NOT
  WORK IN THIS CASE OF OBSTACLE AVOIDING ROBOT.*/

  Q_OLD = Q_TABLE[S][A];                        
  Q_MAX = MAX(Q_TABLE, NEXT_S);
  Q_NEW = (1-LEARNING_RATE)*Q_OLD + LEARNING_RATE*(R + DISCOUNT_FACTOR*Q_MAX); 
  Serial.print("Q VALUE : ");
  Serial.println(Q_NEW);
  Q_TABLE[S][A] = Q_NEW;                          
}

///////////////////////////////////////////////////////////END///////////////////////////////////////////////////////////////


/////////////////////////////////////////START OF MAIN LOOP/////////////////////////////////////////////////
void loop() 
{
  /////////////////////////////////////////TRAINING////////////////////////////////////////////
  for(int I =0; I<EPISODES; I++)
  {
    Serial.println("START :");
    ACTION_TAKEN = false;
    FLAG = 0;
       while(true)
          {
            Forward();
            Obstacle = Obstacle_Avoider();
            if(Obstacle == true)
            {
              NEXT_STATE = STATE+1;
              
              if(NEXT_STATE == 10)
                {
                 NEXT_STATE = 0;
                }

               if(NEXT_STATE < 0)
                {
                 NEXT_STATE = 0;
                }
               
              Serial.print("STATE: ");
              Serial.println(STATE);
              FLAG = 1;
              break;
            }
          }
     if(FLAG ==1)
     {
       PROB = RANDOM(EPSILON);
        if (PROB<=EPSILON)     //EXPLORE THE ACTIONS 
          {
            ACTION = random(0,4);
            FLAG = 2;
          }
        else                  //EXPLOIT THE ACTIONS FROM Q TABLE
          {
            ACTION = ARGMAX(Q,STATE);
            FLAG = 2;
          }
     }   

     if(FLAG ==2)
     {
         if(ACTION == 0)
         {
          Forward();
          delay(1500);
          Stop();
          REWARD = REWARDS[STATE][ACTION];
         }

         if(ACTION == 1)
         {
          Backward();
          delay(2500);
          Stop();
          REWARD = REWARDS[STATE][ACTION];
         }

         if(ACTION == 2)
         {
          Stop();
          REWARD = REWARDS[STATE][ACTION];
         }

         if(ACTION == 3)
         {
          Left();
          delay(2000);
          Stop();
          REWARD = REWARDS[STATE][ACTION];
         }

        ACTION_TAKEN = true;  
        delay(500); 
     }

   if(ACTION_TAKEN == true)
     {
      Update(Q,STATE,NEXT_STATE,ACTION ,ACTIONS,REWARD,ALPHA ,GAMMA);
      STATE = NEXT_STATE;
      EPSILON = DECAY(EPSILON);
      if(EPSILON<0.5)
       {
        EPSILON  == 0.9;
       }
      Serial.print("EPISODE ENDED : ");
      Serial.println(I);
      delay(7000);
     }       
  }
  /////////////////////////////////////END OF TRAINING///////////////////////////////////

 //////////////////////////////////////EVALUATION//////////////////////////////////////////
 /*USE THIS TO CHECK WHETHER YOUR Q VALUES ARE RIGHT OR WRONG. IF ALL Q VALUES ARE 
 COMING RIGHT OR SEEMS RIGHT/ACCURATE COMMENT THIS SECTION */
 for(int y = 0; y<=9 ; y++)
   {
    Serial.println("SET OF Q VALUES  WILL START:");
    for(int l = 0; l<=3; l++)
      {
        Serial.print("Q VALUE :");
        Serial.println(Q[y][l]);
        delay(2000);
      }
     delay(2000);
   }
   Serial.println("EVALUATION ENDED");
////////////////////////////////END OF EVALUATION/////////////////////////////////////////

////////////////////////////////////////TESTING////////////////////////////////////////////
while(true)
 {
  Forward();
  Obstacle = Obstacle_Avoider();
  if(Obstacle == true)
   {
     STATE = GET_STATE();
     ACTION = ARGMAX(Q,STATE);
     Serial.print("ACTION TAKEN: ");
     Serial.println(ACTION);

     if(ACTION ==0)
      {
        Forward();
        delay(1500);
        Stop();
      }

     if(ACTION == 1)
       {
        Backward();
        delay(1500);
        Stop();
       }
     if(ACTION == 2)
       {
        Stop();
       }

     if(ACTION == 3)
       {
        Left();
        delay(2000);
        Stop();
       }
     }
  }
  //////////////////////////////////////////////////END OF TESTING////////////////////////////////////////////////////////////
}
//////////////////////////////////////////////////////END OF MAIN LOOP////////////////////////////////////////////////////////
