// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

import edu.wpi.first.hal.SimBoolean;
import edu.wpi.first.hal.SimDevice;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class DigitalInputWrapper extends DigitalInput{

    private SimBoolean simSensorRdg;
    private SimDevice simDigitalInput;

    public DigitalInputWrapper(int channel) {
        super(channel);

        simDigitalInput  = SimDevice.create("DigitalInput", channel);
     
        if(simDigitalInput != null){
            simSensorRdg = simDigitalInput.createBoolean("Sensor Reading", Direction.kInput, true);
        }

    }

    public boolean get(){
        
        if(simDigitalInput != null){
            return simSensorRdg.get();
        }
        return super.get();
    }

    public void set(boolean val){
        if(simDigitalInput != null){
            simSensorRdg.set(val);
        }
    }







}
