MODULE Module1
        TASK PERS wobjdata Workobject_1:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[942.5,-290,195],[0,0,1,0]]];
    CONST robtarget Target_10:=[[93.75,435,0],[1,0,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_10off:=[[93.75,435,-200],[1,0,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_10giroff:=[[93.75,435,-300],[0,1,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_10giro:=[[93.75,435,-150],[0,1,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20:=[[93.75,145,0],[1,0,0,0],[-1,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20off:=[[93.75,145,-200],[1,0,0,0],[-1,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20giroff:=[[93.75,145,-300],[0,1,0,0],[-1,-1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_20giro:=[[93.75,145,-150],[0,1,0,0],[-1,-1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    PERS tooldata Pinzatula:=[TRUE,[[76,0,272],[0.707106781,0,0.707106781,0]],[1,[0,0,1],[1,0,0,0],0,0,0]];
    CONST robtarget Home:=[[88.649438925,290,-347.682069312],[0.965925826,0,0.258819045,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    TASK PERS wobjdata Workobject_2:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[102.5,625,470],[0,-0.707106781,0.707106781,0]]];
    CONST robtarget Target_100:=[[0,-200,317.5],[1,0,0,0],[0,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_90:=[[0,0,317.5],[1,0,0,0],[0,-1,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget HomeRobot:=[[703.850561075,0,477.682069312],[0.258819045,0,-0.965925826,0],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30:=[[286.25,435,0],[1,0,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30off:=[[286.25,435,-200],[1,0,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30giro:=[[286.25,435,-150],[0,1,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_30giroff:=[[286.25,435,-300],[0,1,0,0],[0,0,-1,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40:=[[286.25,145,0],[1,0,0,0],[-1,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40off:=[[286.25,145,-200],[1,0,0,0],[-1,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40giro:=[[286.25,145,-150],[0,1,0,0],[-1,-1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_40giroff:=[[286.25,145,-300],[0,1,0,0],[-1,-1,-2,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    TASK PERS wobjdata BandaTrans:=[FALSE,TRUE,"",[[0,0,0],[1,0,0,0]],[[920,570,600],[0,0,1,0]]];
    CONST robtarget Target_50:=[[802.5,1000,-365],[0.707106781,0,0,-0.707106781],[0,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_50off:=[[802.5,1000,-500],[0.707106781,0,0,-0.707106781],[0,-1,0,1],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_60:=[[572.5,1000,-365],[0.707106781,0,0,-0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_60off:=[[572.5,1000,-500],[0.707106781,0,0,-0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_70:=[[342.5,1000,-365],[0.707106781,0,0,-0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_70off:=[[342.5,1000,-500],[0.707106781,0,0,-0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_80:=[[112.5,1000,-365],[0.707106781,0,0,-0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_80off:=[[112.5,1000,-500],[0.707106781,0,0,-0.707106781],[0,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    PROC main()
        Reset DO_01;
        Reset DO_02;
        Reset DO_03;
        Reset DO_04;
        Reset DO_05;
        Reset DO_06;
        Set DO_05;
        Reset DO_05;
        HomeRob;
!        WHILE TRUE DO
!            VHome;
            
!            IF DI_01 = 1 THEN
!                Toma1;
!            ENDIF
            
!            IF DI_02 = 1 THEN
!                Arepa1;
!            ENDIF
            
!            IF DI_03 = 1 THEN
!                Saca1;
!            ENDIF
            
!            !Rutinas de voltear arepas
!            IF V1 = 1 THEN
!                Arepa1;
!            ENDIF
!            IF V2 = 1 THEN
!                Arepa2;
!            ENDIF
!            IF V3 = 1 THEN
!                Arepa3;
!            ENDIF
!            IF V4 = 1 THEN
!                Arepa4;
!            ENDIF
            
!            !Rutinas de Tomar arepas
!            IF T1 = 1 THEN
!                Toma1;
!            ENDIF
!            IF T2 = 1 THEN
!                Toma2;
!            ENDIF
!            IF T3 = 1 THEN
!                Toma3;
!            ENDIF
!            IF T4 = 1 THEN
!                Toma4;
!            ENDIF
            
!            !Rutinas de sacar arepas
!            IF S1 = 1 THEN
!                Saca1;
!            ENDIF
!            IF S2 = 1 THEN
!                Saca2;
!            ENDIF
!            IF S3 = 1 THEN
!                Saca3;
!            ENDIF
!            IF S4 = 1 THEN
!                Saca4;
!            ENDIF
            
!        ENDWHILE
        !MoveL HomeRobot,v1000,z100,Pinzatula\WObj:=wobj0;
    ENDPROC
    
    PROC Arepa1()
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveJ Target_10off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
        MoveL Target_10,v50,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveL Target_10off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_10giroff,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_10giro,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
    ENDPROC
    PROC VHome()
        MoveJ Home,v100,z10,Pinzatula\WObj:=Workobject_1;
        
    ENDPROC
    PROC Arepa2()
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveJ Target_20off,v100,z10,Pinzatula\WObj:=Workobject_1;
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
        MoveL Target_20,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveL Target_20off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveL Target_20giroff,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveL Target_20giro,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
    ENDPROC
    PROC Path_10()
    ENDPROC
    PROC Path_20()
        MoveL HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
    ENDPROC
    PROC HomeRob()
        MoveL HomeRobot,v100,z0,Pinzatula\WObj:=wobj0;
    ENDPROC
    PROC Arepa3()
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveJ Target_30off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
        MoveL Target_30,v50,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveL Target_30off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_30giroff,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_30giro,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
    ENDPROC
    PROC Arepa4()
        Set DO_06;
        WaitTime(2);
        Reset DO_06;
        MoveJ Target_40off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveL Target_40,v50,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveL Target_40off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_40giroff,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_40giro,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(2);
        Set DO_05;
        WaitTime(2);
        Reset DO_05;
    ENDPROC
    PROC Toma1()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_50off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_50,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_50off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_10off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_10,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
    ENDPROC
    PROC Toma2()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_60off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_60,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_60off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_20off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_20,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
    ENDPROC
    PROC Toma3()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_70off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_70,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_70off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_30off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_30,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
    ENDPROC
    PROC Toma4()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_80off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_80,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_80off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_10off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_10,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
    ENDPROC
    
    
    PROC Saca1()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_10off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_10,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;      
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_50off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_50,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_50off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
    ENDPROC
    
    PROC Saca2()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_20off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_20,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;      
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_60off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_60,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_60off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
    ENDPROC
    
    PROC Saca3()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_30off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_30,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;      
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_70off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_70,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_70off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
    ENDPROC
    
    PROC Saca4()
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;
        MoveJ Target_40off,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_40,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Cierra
        Set DO_06;
        WaitTime(1);
        Reset DO_06;      
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
        MoveJ Target_80off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ Target_80,v100,z10,Pinzatula\WObj:=Workobject_1;
        WaitTime(1); !Abre
        Set DO_05;
        WaitTime(1);
        Reset DO_05;
        MoveJ Target_80off,v100,z10,Pinzatula\WObj:=Workobject_1;
        MoveJ HomeRobot,v100,z10,Pinzatula\WObj:=wobj0;
    ENDPROC
    PROC Path_30()
        MoveL HomeRobot,v1000,z100,Pinzatula\WObj:=wobj0;
        MoveL Target_50,v1000,z100,Pinzatula\WObj:=Workobject_1;
        MoveL Target_10,v1000,z100,Pinzatula\WObj:=Workobject_1;
    ENDPROC
    PROC Path_40()
        MoveL HomeRobot,v1000,z100,Pinzatula\WObj:=wobj0;
        MoveL Target_60,v1000,z100,Pinzatula\WObj:=Workobject_1;
        MoveL Target_20,v1000,z100,Pinzatula\WObj:=Workobject_1;
    ENDPROC
    PROC Path_50()
        MoveL Target_70,v1000,z100,Pinzatula\WObj:=Workobject_1;
    ENDPROC
    PROC Path_60()
        MoveL Target_80,v1000,z100,Pinzatula\WObj:=Workobject_1;
    ENDPROC
ENDMODULE