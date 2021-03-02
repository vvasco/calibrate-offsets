# Copyright: (C) 2021 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Vadim Tikhanoff
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
#
# calibOffsets.thrift

/**
* calibOffsets_IDL
*
* IDL Interface to \ref Calib Offsets Module.
*/
service calibOffsets_IDL
{
    /**
     * Quit the module.
     * @return true/false on success/failure
    */
    bool quit();

    /**
     * Calibrate arm.
     * @param part to calibrate.
     * @return true/false on success/failure
    */
    bool calibrate(1:string part);

    /**
     * Home arms and gaze.
     * @return true/false on success/failure
    */
    bool home();

    /**
     * Get reaching offset.
     * @return reaching offset (x, y, z) with respect to robot root.
    */
    list<double> getOffset();

}
