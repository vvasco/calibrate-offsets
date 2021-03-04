/*
 * Copyright (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
 * Author: Vadim Tikhanoff
 * email:  vadim.tikhanoff@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

#include <yarp/os/BufferedPort.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/RFModule.h>
#include <yarp/os/Network.h>
#include <yarp/os/Log.h>
#include <yarp/os/Time.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Semaphore.h>
#include <yarp/os/RpcClient.h>

#include <yarp/sig/Image.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/GazeControl.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IPositionControl.h>

#include <yarp/math/Math.h>

#include <sstream>
#include <string>
#include <fstream>

#include "calibOffsets_IDL.h"

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::os::Bottle >
{
    std::string moduleName;
    std::string robotName;
    yarp::sig::Vector calibLeft, calibRight, calibLeftPos;
    yarp::sig::Vector homePos, homeVels;
    double skinPressureThresh;
    int activeTaxelsThresh;
    double ballLikelihoodThresh;

    yarp::os::BufferedPort<yarp::os::Bottle > trackerInPort;

    std::string part;
    bool calibrating;
    std::vector<double> offset;

    yarp::dev::PolyDriver *drvCartLeftArm;
    yarp::dev::PolyDriver *drvCartRightArm;
    yarp::dev::PolyDriver *drvLeftArm;
    yarp::dev::PolyDriver *drvRightArm;
    yarp::dev::PolyDriver *drvGaze;
    yarp::dev::IPositionControl *iposLeft;
    yarp::dev::IControlMode *imodeLeft;
    yarp::dev::IPositionControl *iposRight;
    yarp::dev::IControlMode *imodeRight;
    yarp::dev::ICartesianControl *icartLeft;
    yarp::dev::ICartesianControl *icartRight;
    yarp::dev::IGazeControl *igaze;

    std::mutex mtx;

public:
    /********************************************************/

    Processing( const std::string &moduleName, const std::string &robotName,
                yarp::os::Bottle *cl, yarp::os::Bottle *cr,
                yarp::os::Bottle *homep, yarp::os::Bottle *homev,
                const double &skinPressureThresh, const int &activeTaxelsThresh,
                const double &ballLikelihoodThresh, yarp::os::Bottle *calibLeftPosition)
    {
        this->moduleName = moduleName;
        this->robotName = robotName;

        if (cl->size() > 0)
        {
            calibLeft.resize(7);
            calibLeft[0] = cl->get(0).asDouble();
            calibLeft[1] = cl->get(1).asDouble();
            calibLeft[2] = cl->get(2).asDouble();
            calibLeft[3] = cl->get(3).asDouble();
            calibLeft[4] = cl->get(4).asDouble();
            calibLeft[5] = cl->get(5).asDouble();
            calibLeft[6] = cl->get(6).asDouble();
        }

        if (cr->size() > 0)
        {
            calibRight.resize(7);
            calibRight[0] = cr->get(0).asDouble();
            calibRight[1] = cr->get(1).asDouble();
            calibRight[2] = cr->get(2).asDouble();
            calibRight[3] = cr->get(3).asDouble();
            calibRight[4] = cr->get(4).asDouble();
            calibRight[5] = cr->get(5).asDouble();
            calibRight[6] = cr->get(6).asDouble();
        }

        if (homep->size() > 0)
        {
            homePos.resize(7);
            homePos[0] = homep->get(0).asDouble();
            homePos[1] = homep->get(1).asDouble();
            homePos[2] = homep->get(2).asDouble();
            homePos[3] = homep->get(3).asDouble();
            homePos[4] = homep->get(4).asDouble();
            homePos[5] = homep->get(5).asDouble();
            homePos[6] = homep->get(6).asDouble();
        }

        if (homev->size() > 0)
        {
            homeVels.resize(7);
            homeVels[0] = homev->get(0).asDouble();
            homeVels[1] = homev->get(1).asDouble();
            homeVels[2] = homev->get(2).asDouble();
            homeVels[3] = homev->get(3).asDouble();
            homeVels[4] = homev->get(4).asDouble();
            homeVels[5] = homev->get(5).asDouble();
            homeVels[6] = homev->get(6).asDouble();
        }
        
         if (calibLeftPosition->size() > 0)
        {
            calibLeftPos.resize(7);
            calibLeftPos[0] = calibLeftPosition->get(0).asDouble();
            calibLeftPos[1] = calibLeftPosition->get(1).asDouble();
            calibLeftPos[2] = calibLeftPosition->get(2).asDouble();
            calibLeftPos[3] = calibLeftPosition->get(3).asDouble();
            calibLeftPos[4] = calibLeftPosition->get(4).asDouble();
            calibLeftPos[5] = calibLeftPosition->get(5).asDouble();
            calibLeftPos[6] = calibLeftPosition->get(6).asDouble();
        }

        this->skinPressureThresh = skinPressureThresh;
        this->activeTaxelsThresh = activeTaxelsThresh;
        this->ballLikelihoodThresh = ballLikelihoodThresh;
     
    }

    /********************************************************/
    ~Processing()
    {
    };

    /********************************************************/
    bool open()
    {

        this->useCallback();

        BufferedPort<yarp::os::Bottle >::open( "/" + moduleName + "/handSkin:i" );
        trackerInPort.open("/" + moduleName + "/tracker:i");

        calibrating = false;
        offset.resize(3);
        iposLeft = NULL;
        imodeLeft = NULL;
        iposRight = NULL;
        imodeRight = NULL;
        icartLeft = NULL;
        icartRight = NULL;
        igaze = NULL;

        // CARTESIAN LEFT
        yarp::os::Property optCartLeftArm("(device cartesiancontrollerclient)");
        optCartLeftArm.put("remote", "/" + robotName + "/cartesianController/left_arm");
        optCartLeftArm.put("local", "/" + moduleName + "/cartesian/left_arm");
        drvCartLeftArm=new yarp::dev::PolyDriver;
        if (!drvCartLeftArm->open(optCartLeftArm))
        {
            yError() << "Could not open left cartesian";
            return false;
        }

        // CARTESIAN RIGHT
        yarp::os::Property optCartRightArm("(device cartesiancontrollerclient)");
        optCartRightArm.put("remote", "/" + robotName + "/cartesianController/right_arm");
        optCartRightArm.put("local", "/" + moduleName + "/cartesian/right_arm");
        drvCartRightArm=new yarp::dev::PolyDriver;
        if (!drvCartRightArm->open(optCartRightArm))
        {
            yError() << "Could not open right cartesian";
            return false;
        }

        // GAZE
        yarp::os::Property optGazeCtrl("(device gazecontrollerclient)");
        optGazeCtrl.put("remote", "/iKinGazeCtrl");
        optGazeCtrl.put("local", "/" + moduleName + "/gaze");
        drvGaze=new yarp::dev::PolyDriver;
        if (!drvGaze->open(optGazeCtrl))
        {
            yError() << "Could not open gaze";
            return false;
        }

        // CONTROLBOARD LEFT
        yarp::os::Property optLeftArm("(device remote_controlboard)");
        optLeftArm.put("remote",  "/" + robotName + "/left_arm");
        optLeftArm.put("local", "/" + moduleName + "/left_arm");
        drvLeftArm=new yarp::dev::PolyDriver;
        if (!drvLeftArm->open(optLeftArm))
        {
            yError() << "Could not open left arm";
            return false;
        }

        // CONTROLBOARD RIGHT
        yarp::os::Property optRightArm("(device remote_controlboard)");
        optRightArm.put("remote",  "/" + robotName + "/right_arm");
        optRightArm.put("local", "/" + moduleName + "/right_arm");
        drvRightArm=new yarp::dev::PolyDriver;
        if (!drvRightArm->open(optRightArm))
        {
            yError() << "Could not open right arm";
            return false;
        }

        if (drvCartLeftArm->isValid() && drvGaze->isValid() && drvRightArm->isValid()
                && drvLeftArm->isValid() && drvRightArm->isValid())
        {            
            drvLeftArm->view(iposLeft);
            drvLeftArm->view(imodeLeft);
            drvRightArm->view(iposRight);
            drvRightArm->view(imodeRight);
            drvCartLeftArm->view(icartLeft);
            drvCartRightArm->view(icartRight);
            drvGaze->view(igaze);
        }
        else
        {
            yError() << "Could not open arm / gaze interface";
            return false;
        }
        
        for (size_t j=0; j<homeVels.length(); j++)
        {
            imodeLeft->setControlMode(j,VOCAB_CM_POSITION);
            imodeRight->setControlMode(j,VOCAB_CM_POSITION);
            
            iposLeft->setRefSpeed(j,homeVels[j]);
            iposRight->setRefSpeed(j,homeVels[j]);
        }
        
        return true;
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle >::close();
        trackerInPort.close();

        if (drvCartLeftArm)
        {
            delete drvCartLeftArm;
        }
        if (drvCartRightArm)
        {
            delete drvCartRightArm;
        }
        if (drvLeftArm)
        {
            delete drvLeftArm;
        }
        if (drvRightArm)
        {
            delete drvRightArm;
        }
        if (drvGaze)
        {
            delete drvGaze;
        }
    }

    /********************************************************/
    void interrupt()
    {
        home();
        BufferedPort<yarp::os::Bottle >::interrupt();
        trackerInPort.interrupt();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &inSkin )
    {
        std::lock_guard<std::mutex> lg(mtx);
        for (int j=0; j < inSkin.size(); j++)
        {
            yarp::os::Bottle *subSkin = inSkin.get(j).asList();
            if (subSkin->size() > 0)
            {
                yarp::os::Bottle *bodyPart = subSkin->get(0).asList();
                bool ok_to_go = false;
                if (part == "left")
                {
                    ok_to_go = bodyPart->get(1).asInt() == 3 && bodyPart->get(2).asInt() == 6
                            && bodyPart->get(3).asInt() == 1;
                }
                else if (part == "right")
                {
                    ok_to_go = bodyPart->get(1).asInt() == 4 && bodyPart->get(2).asInt() == 6
                            && bodyPart->get(3).asInt() == 4;
                }
                else
                {
                    yInfo() << "Not handled";
                }
//                bool ok_to_go = true;
                if (ok_to_go && calibrating)
                {
                    double avgPressure = subSkin->get(7).asDouble();
                    if (avgPressure >= skinPressureThresh)
                    {
                        yarp::os::Bottle *activeTaxels = subSkin->get(6).asList();
                        int countActive = 0;
                        for (int i = 0; i < activeTaxels->size(); i++)
                        {
                            int ai = activeTaxels->get(i).asInt();
                            if (ai >= 97 && ai <= 144)
                            {
                                countActive++;
                            }
                        }
                        yInfo() << "Found" << countActive << "palm active taxels";

//                        int countActive = 4;
                        if (countActive >= activeTaxelsThresh)
                        {
                            yarp::os::Bottle *ballPos = trackerInPort.read();
                            if (ballPos->size() > 0)
                            {
                                if (ballPos->get(3).asDouble() > ballLikelihoodThresh)
                                {
                                    yarp::sig::Vector xEye, oEye;
                                    igaze->getLeftEyePose(xEye, oEye);
                                    yarp::sig::Matrix eye2root = yarp::math::axis2dcm(oEye);
                                    eye2root.setSubcol(xEye, 0, 3);

                                    yarp::sig::Vector posBallEye(4);
                                    posBallEye[0] = ballPos->get(0).asDouble();
                                    posBallEye[1] = ballPos->get(1).asDouble();
                                    posBallEye[2] = ballPos->get(2).asDouble();
                                    posBallEye[3] = 1.0;

                                    yarp::sig::Vector posBallRoot = eye2root * posBallEye;
                                    posBallRoot.pop_back();
                                    yDebug() << "Ball pos root" << posBallRoot.toString();

                                    yarp::sig::Vector xHand;
                                    yarp::sig::Vector oHand;
                                    icartLeft->getPose(xHand, oHand);
                                    yDebug() << "Hand Effector" << xHand.toString();

                                    offset[0] = posBallRoot[0] - xHand[0];
                                    offset[1] = posBallRoot[1] - xHand[1];
                                    offset[2] = posBallRoot[2] - xHand[2];
                                    yDebug() << "Offset" << offset[0] << offset[1] << offset[2];
                                    calibrating = false;
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /**********************************************************/
    std::vector<double> getOffset()
    {
        std::lock_guard<std::mutex> lg(mtx);
        return offset;
    }

    /**********************************************************/
    bool calibrate(const std::string part)
    {
        std::lock_guard<std::mutex> lg(mtx);
        this->part = part;
        yarp::sig::Vector xd(3);
        yarp::sig::Vector od(4);
        yarp::dev::ICartesianControl *icart = NULL;
        if (part == "left")
        {
            xd[0] = calibLeft[0];
            xd[1] = calibLeft[1];
            xd[2] = calibLeft[2];
            od[0] = calibLeft[3];
            od[1] = calibLeft[4];
            od[2] = calibLeft[5];
            od[3] = calibLeft[6];
            icart = icartLeft;
        }
        if (part == "right")
        {
            xd[0] = calibRight[0];
            xd[1] = calibRight[1];
            xd[2] = calibRight[2];
            od[0] = calibRight[3];
            od[1] = calibRight[4];
            od[2] = calibRight[5];
            od[3] = calibRight[6];
            icart = icartRight;
        }
        
      
        for (size_t j=0; j<calibLeftPos.length(); j++)
        {
            iposLeft->setRefSpeed(j,homeVels[j]);
	    iposLeft->positionMove(j,calibLeftPos[j]);
	}
        //if (!icart->goToPoseSync(xd, od))
        //{
        //   yError() << part << "arm could not reach" << xd.toString();
        //  return false;
        //}
        //icart->waitMotionDone(0.001, 5.0);
        
	yarp::sig::Vector x0,o0;
	bool done = false;
	
	while (done==false){
	    iposLeft->checkMotionDone(&done);
	    yarp::os::Time::delay(0.1);
	    yInfo() << "Done" << done;
	}
	icart->getPose(x0,o0);

        if (!igaze->lookAtFixationPointSync(x0))
        {
            yError() << "Could not fixate" << x0.toString();
            return false;
        }
        igaze->waitMotionDone(0.001, 5.0);

        calibrating = true;
        return true;
    }

    /**********************************************************/
    bool home()
    {
        std::lock_guard<std::mutex> lg(mtx);
        yInfo() << "Homing arms and gaze";
        yarp::sig::Vector xd(3, 0.0);
        xd[0] = -1.0;
        xd[2] = 0.3;
        if (!igaze->lookAtFixationPoint(xd))
        {
            yError() << "Could not fixate" << xd.toString();
            return false;
        }

        //for (size_t j=0; j<homeVels.length(); j++)
        //{
        //    imodeLeft->setControlMode(j,VOCAB_CM_POSITION);
        //    imodeRight->setControlMode(j,VOCAB_CM_POSITION);
        //}

        for (size_t j=0; j<homeVels.length(); j++)
        {
            //iposLeft->setRefSpeed(j,homeVels[j]);
            iposLeft->positionMove(j,homePos[j]);
            //iposRight->setRefSpeed(j,homeVels[j]);
            iposRight->positionMove(j,homePos[j]);
        }
        return true;
    }
};

/********************************************************/
class Module : public yarp::os::RFModule, public calibOffsets_IDL
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

    /********************************************************/
    bool attach(yarp::os::RpcServer &source)
    {
        return this->yarp().attachAsServer(source);
    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("calibOffsets"), "module name (string)").asString();
        setName(moduleName.c_str());

        std::string robotName = rf.check("robot", yarp::os::Value("icub"), "robot name (string)").asString();

        if (!rf.check("calibLeft") || !rf.check("calibRight"))
        {
            yError() << "Could not find calibLeft or calibRight";
            return false;
        }

        if (!rf.check("homePos") || !rf.check("homeVels"))
        {
            yError() << "Could not find homePos or homeVels";
            return false;
        }
       
        if (!rf.check("calibLeftPosition"))
        {
            yError() << "Could not find calibLeftPosition";
            return false;
        }

        yarp::os::Bottle *calibLeft=rf.find("calibLeft").asList();
        yarp::os::Bottle *calibRight=rf.find("calibRight").asList();
        yarp::os::Bottle *homePos=rf.find("homePos").asList();
        yarp::os::Bottle *homeVels=rf.find("homeVels").asList();
        yarp::os::Bottle *calibLeftPosition=rf.find("calibLeftPosition").asList();

        double skinPressureThresh = rf.check("skinPressureThresh", yarp::os::Value(20.0), "threshold for skin average pressure").asDouble();
        int activeTaxelsThresh = rf.check("activeTaxelsThresh", yarp::os::Value(3), "threshold for palm active taxels").asInt();
        double ballLikelihoodThresh = rf.check("ballLikelihoodThresh", yarp::os::Value(0.0005), "threshold on likelihood for detecting the ball").asDouble();

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;
        processing = new Processing( moduleName, robotName, calibLeft, calibRight, homePos, homeVels,
                                     skinPressureThresh, activeTaxelsThresh, ballLikelihoodThresh, calibLeftPosition );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        rpcPort.close();
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool calibrate(const std::string &part) override
    {
        return processing->calibrate(part);
    }

    /**********************************************************/
    std::vector<double> getOffset() override
    {
        return processing->getOffset();
    }

    /**********************************************************/
    bool home() override
    {
        return processing->home();
    }

    /**********************************************************/
    bool quit() override
    {
        closing = true;
        return true;
    }

    /********************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /********************************************************/
    bool updateModule()
    {
        return !closing;
    }

};

/********************************************************/
int main(int argc, char *argv[])
{
    yarp::os::Network::init();

    yarp::os::Network yarp;
    if (!yarp.checkNetwork())
    {
        yError("YARP server not available!");
        return 1;
    }

    Module module;
    yarp::os::ResourceFinder rf;

    rf.setVerbose();
    rf.setDefaultContext("calibOffsets");
    rf.setVerbose();
    rf.setDefaultContext(rf.getContext());
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc,argv);

    return module.runModule(rf);
}
//empty line to make gcc happy
