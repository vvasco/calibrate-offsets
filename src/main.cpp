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

#include <yarp/math/Math.h>

#include <sstream>
#include <string>
#include <fstream>

/********************************************************/
class Processing : public yarp::os::BufferedPort<yarp::os::Bottle >
{
    std::string moduleName;

    yarp::os::RpcServer handlerPort;

    yarp::os::BufferedPort<yarp::os::Bottle > trackerInPort;

    yarp::os::RpcClient rpcClient;

    yarp::dev::PolyDriver *drvLeftArm;
    yarp::dev::PolyDriver *drvGaze;
    yarp::dev::ICartesianControl *icart;
    yarp::dev::IGazeControl *igaze;

    std::mutex mtx;

public:
    /********************************************************/

    Processing( const std::string &moduleName)
    {
        this->moduleName = moduleName;
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

        yarp::os::Property optLeftArm("(device cartesiancontrollerclient)");
        optLeftArm.put("remote", "/icub/cartesianController/left_arm");
        optLeftArm.put("local", "/" + moduleName + "/left_arm");
        drvLeftArm=new yarp::dev::PolyDriver;
        if (!drvLeftArm->open(optLeftArm))
        {
            yError() << "Could not open left arm";
            close();
            return false;
        }

        yarp::os::Property optGazeCtrl("(device gazecontrollerclient)");
        optGazeCtrl.put("remote", "/iKinGazeCtrl");
        optGazeCtrl.put("local", "/" + moduleName + "/gaze");
        drvGaze=new yarp::dev::PolyDriver;
        if (!drvGaze->open(optGazeCtrl))
        {
            yError() << "Could not open gaze";
            close();
            return false;
        }

        if (drvLeftArm->isValid() && drvGaze->isValid())
        {
            drvLeftArm->view(icart);
            drvGaze->view(igaze);
        }
        else
        {
            yError() << "Could not open arm / gaze interface";
            close();
            return false;
        }

        return true;
    }

    /********************************************************/
    void close()
    {
        BufferedPort<yarp::os::Bottle >::close();
        trackerInPort.close();

        delete drvLeftArm;
        delete drvGaze;
        delete icart;
        delete igaze;
    }

    /********************************************************/
    void interrupt()
    {
        BufferedPort<yarp::os::Bottle >::interrupt();
        trackerInPort.interrupt();
    }

    /********************************************************/
    void onRead( yarp::os::Bottle &inSkin )
    {
        for (int j=0; j < inSkin.size(); j++)
        {
            yarp::os::Bottle *subSkin = inSkin.get(j).asList();
            if (subSkin->size() > 0)
            {
                yarp::os::Bottle *bodyPart = subSkin->get(0).asList();
                if (bodyPart->get(1).asInt() == 3 && bodyPart->get(2).asInt() == 6 && bodyPart->get(3).asInt() == 1)
                {
//                    yInfo() << "Detected left hand";
                    double avgPressure = subSkin->get(7).asDouble();
                    if (avgPressure >= 20.0)
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
//                        yInfo() << "Found" << countActive << "palm active taxels";

                        int countActive = 4;
                        if (countActive >= 3)
                        {
                            yarp::os::Bottle *ballPos = trackerInPort.read();
                            if (ballPos->size() > 0)
                            {
                                if (ballPos->get(3).asDouble() > 0.0005)
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
                                    icart->getPose(xHand, oHand);
                                    yDebug() << "Hand Effector" << xHand.toString();

                                    yarp::sig::Vector offset = posBallRoot - xHand;
                                    yDebug() << "Offset" << offset.toString();
                                }
                            }
                        }
                    }
                }
            }
        }
    }
};

/********************************************************/
class Module : public yarp::os::RFModule
{
    yarp::os::ResourceFinder    *rf;
    yarp::os::RpcServer         rpcPort;

    Processing                  *processing;
    friend class                processing;

    bool                        closing;

//    /********************************************************/
//    bool attach(yarp::os::RpcServer &source)
//    {
//        return this->yarp().attachAsServer(source);
//    }

public:

    /********************************************************/
    bool configure(yarp::os::ResourceFinder &rf)
    {
        this->rf=&rf;
        std::string moduleName = rf.check("name", yarp::os::Value("calibOffsets"), "module name (string)").asString();
        setName(moduleName.c_str());

        rpcPort.open(("/"+getName("/rpc")).c_str());

        closing = false;


        processing = new Processing( moduleName );

        /* now start the thread to do the work */
        processing->open();

        attach(rpcPort);

        return true;
    }

    /**********************************************************/
    bool close()
    {
        processing->interrupt();
        processing->close();
        delete processing;
        return true;
    }

    /**********************************************************/
    bool quit()
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
    rf.configure(argc,argv);

    return module.runModule(rf);
}
//empty line to make gcc happy
