// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include <assert.h>
#include <iostream>
#include <fstream>
#include <chrono>
#include <future>

#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <math.h>

# define M_PI 3.141592653589793238462643383279502884

using namespace std;

void save_body_information(k4abt_body_t body, ofstream &file_input, float x_angle, float y_angle, float z_angle)
{
    file_input << chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count() << ',' << body.id << ',';
    for (int i = 0; i < (int)K4ABT_JOINT_COUNT; i++)
    {
        k4a_float3_t position = body.skeleton.joints[i].position;
        k4a_quaternion_t orientation = body.skeleton.joints[i].orientation;
        k4abt_joint_confidence_level_t confidence_level = body.skeleton.joints[i].confidence_level;
        float y_rot = position.v[1] * cos(x_angle * M_PI/180) + position.v[2] * sin(x_angle * M_PI/180);
        float z_rot = position.v[2] * cos(x_angle * M_PI/180) - position.v[1] * sin(x_angle * M_PI/180);
        float x_rot = position.v[0] * cos(y_angle * M_PI/180) + z_rot * sin(y_angle * M_PI/180);
        float z_final = z_rot * cos(y_angle * M_PI/180) - position.v[0] * sin(y_angle * M_PI/180);
        float x_final = x_rot * cos(z_angle * M_PI/180) + y_rot * sin(z_angle * M_PI/180);
        float y_final = y_rot * cos(z_angle * M_PI/180) - x_rot * sin(z_angle * M_PI/180);
        file_input << position.v[0] << ',' << position.v[1] << ',' << position.v[2] << ',' << x_final << ',' << y_final << ',' << z_final << ',' << confidence_level << ',';
    }
    file_input << '\n';
}

void print_body_index_map_middle_line(k4a::image body_index_map)
{
    uint8_t* body_index_map_buffer = body_index_map.get_buffer();

    // Given body_index_map pixel type should be uint8, the stride_byte should be the same as width
    // TODO: Since there is no API to query the byte-per-pixel information, we have to compare the width and stride to
    // know the information. We should replace this assert with proper byte-per-pixel query once the API is provided by
    // K4A SDK.
    assert(body_index_map.get_stride_bytes() == body_index_map.get_width_pixels());

    int middle_line_num = body_index_map.get_height_pixels() / 2;
    body_index_map_buffer = body_index_map_buffer + middle_line_num * body_index_map.get_width_pixels();

    cout << "BodyIndexMap at Line " << middle_line_num << ":" << endl;
    for (int i = 0; i < body_index_map.get_width_pixels(); i++)
    {
        cout << (int)*body_index_map_buffer << ", ";
        body_index_map_buffer++;
    }
    cout << endl;
}

void get_body_data(k4abt::tracker &trk, k4a::capture &sensor_capture, ofstream &file, long long old_time, float x_angle, float y_angle, float z_angle) {
    if (!trk.enqueue_capture(sensor_capture))
    {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        throw new exception("Add capture to tracker process queue timeout!");
    }

    k4abt::frame body_frame = trk.pop_result();
    cout << "Results obtained\n";
    cout << "Timer: " << (chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count() - old_time) << " ms\n";

    if (body_frame != nullptr)
    {
        uint32_t num_bodies = body_frame.get_num_bodies();
        cout << num_bodies << " bodies are detected!" << endl;
        for (uint32_t i = 0; i < num_bodies; i++)
        {
            k4abt_body_t body = body_frame.get_body(i);
            save_body_information(body, file, x_angle, y_angle, z_angle);
        }
        cout << "Saved body information\n";

        k4a::image body_index_map = body_frame.get_body_index_map();
        if (body_index_map == nullptr) {
            cout << "Error: Failed to generate body index map!" << endl;
        }
    }
    else
    {
        // It should never hit timeout when K4A_WAIT_INFINITE is set.
        throw new exception("Error! Pop body frame result timeout!");
    }
}
int main()
{
    ios::sync_with_stdio(false);

    try
    {
        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;

        float fps = 30;
        if (fps > 30){
            throw new exception("Frames per second above maximum (30)");
        }else{
            device_config.camera_fps = K4A_FRAMES_PER_SECOND_30;
        }

        k4a::device device = k4a::device::open(0);
        device.start_cameras(&device_config);

        k4a::calibration sensor_calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution);

        k4abt::tracker trk = k4abt::tracker::create(sensor_calibration);

        ofstream file;
        string filename;
        string discard;
        string option;
        float length_s;
        float x_angle;
        float y_angle;
        float z_angle;

        cout << "Enter destination file: ";
        cin >> filename;

        cout << "Enter camera angle around X axis in degrees: ";
        cin >> x_angle;
        cout << "Enter camera angle around Y axis in degrees: ";
        cin >> y_angle;
        cout << "Enter camera angle around Z axis in degrees: ";
        cin >> z_angle;


        cout << "Enter desired length of recording in seconds: ";
        cin >> length_s;
        
        cout << "Enter some text to start recording. ";
        cin >> discard;
        
        file.open(filename, ofstream::out | ofstream::trunc);
        file << "Timestamp,Body,Pelvis_X,Pelvis_Y,Pelvis_Z,Pelvis_RotX,Pelvis_RotY,Pelvis_RotZ,Pelvis_confidence,SpineNaval_X,SpineNaval_Y,SpineNaval_Z,SpineNaval_RotX,SpineNaval_RotY,SpineNaval_RotZ,SpineNaval_confidence,SpineChest_X,SpineChest_Y,SpineChest_Z,SpineChest_RotX,SpineChest_RotY,SpineChest_RotZ,SpineChest_confidence,Neck_X,Neck_Y,Neck_Z,Neck_RotX,Neck_RotY,Neck_RotZ,Neck_confidence,ClavicleLeft_X,ClavicleLeft_Y,ClavicleLeft_Z,ClavicleLeft_RotX,ClavicleLeft_RotY,ClavicleLeft_RotZ,ClavicleLeft_confidence,ShoulderLeft_X,ShoulderLeft_Y,ShoulderLeft_Z,ShoulderLeft_RotX,ShoulderLeft_RotY,ShoulderLeft_RotZ,ShoulderLeft_confidence,ElbowLeft_X,ElbowLeft_Y,ElbowLeft_Z,ElbowLeft_RotX,ElbowLeft_RotY,ElbowLeft_RotZ,ElbowLeft_confidence,WristLeft_X,WristLeft_Y,WristLeft_Z,WristLeft_RotX,WristLeft_RotY,WristLeft_RotZ,WristLeft_confidence,HandLeft_X,HandLeft_Y,HandLeft_Z,HandLeft_RotX,HandLeft_RotY,HandLeft_RotZ,HandLeft_confidence,HandtipLeft_X,HandtipLeft_Y,HandtipLeft_Z,HandtipLeft_RotX,HandtipLeft_RotY,HandtipLeft_RotZ,HandtipLeft_confidence,ThumbLeft_X,ThumbLeft_Y,ThumbLeft_Z,ThumbLeft_RotX,ThumbLeft_RotY,ThumbLeft_RotZ,ThumbLeft_confidence,ClavicleRight_X,ClavicleRight_Y,ClavicleRight_Z,ClavicleRight_RotX,ClavicleRight_RotY,ClavicleRight_RotZ,ClavicleRight_confidence,ShoulderRight_X,ShoulderRight_Y,ShoulderRight_Z,ShoulderRight_RotX,ShoulderRight_RotY,ShoulderRight_RotZ,ShoulderRight_confidence,ElbowRight_X,ElbowRight_Y,ElbowRight_Z,ElbowRight_RotX,ElbowRight_RotY,ElbowRight_RotZ,ElbowRight_confidence,WristRight_X,WristRight_Y,WristRight_Z,WristRight_RotX,WristRight_RotY,WristRight_RotZ,WristRight_confidence,HandRight_X,HandRight_Y,HandRight_Z,HandRight_RotX,HandRight_RotY,HandRight_RotZ,HandRight_confidence,HandtipRight_X,HandtipRight_Y,HandtipRight_Z,HandtipRight_RotX,HandtipRight_RotY,HandtipRight_RotZ,HandtipRight_confidence,ThumbRight_X,ThumbRight_Y,ThumbRight_Z,ThumbRight_RotX,ThumbRight_RotY,ThumbRight_RotZ,ThumbRight_confidence,HipLeft_X,HipLeft_Y,HipLeft_Z,HipLeft_RotX,HipLeft_RotY,HipLeft_RotZ,HipLeft_confidence,KneeLeft_X,KneeLeft_Y,KneeLeft_Z,KneeLeft_RotX,KneeLeft_RotY,KneeLeft_RotZ,KneeLeft_confidence,AnkleLeft_X,AnkleLeft_Y,AnkleLeft_Z,AnkleLeft_RotX,AnkleLeft_RotY,AnkleLeft_RotZ,AnkleLeft_confidence,FootLeft_X,FootLeft_Y,FootLeft_Z,FootLeft_RotX,FootLeft_RotY,FootLeft_RotZ,FootLeft_confidence,HipRight_X,HipRight_Y,HipRight_Z,HipRight_RotX,HipRight_RotY,HipRight_RotZ,HipRight_confidence,KneeRight_X,KneeRight_Y,KneeRight_Z,KneeRight_RotX,KneeRight_RotY,KneeRight_RotZ,KneeRight_confidence,AnkleRight_X,AnkleRight_Y,AnkleRight_Z,AnkleRight_RotX,AnkleRight_RotY,AnkleRight_RotZ,AnkleRight_confidence,FootRight_X,FootRight_Y,FootRight_Z,FootRight_RotX,FootRight_RotY,FootRight_RotZ,FootRight_confidence,Head_X,Head_Y,Head_Z,Head_RotX,Head_RotY,Head_RotZ,Head_confidence,Nose_X,Nose_Y,Nose_Z,Nose_RotX,Nose_RotY,Nose_RotZ,Nose_confidence,EyeLeft_X,EyeLeft_Y,EyeLeft_Z,EyeLeft_RotX,EyeLeft_RotY,EyeLeft_RotZ,EyeLeft_confidence,EarLeft_X,EarLeft_Y,EarLeft_Z,EarLeft_RotX,EarLeft_RotY,EarLeft_RotZ,EarLeft_confidence,EyeRight_X,EyeRight_Y,EyeRight_Z,EyeRight_RotX,EyeRight_RotY,EyeRight_RotZ,EyeRight_confidence,EarRight_X,EarRight_Y,EarRight_Z,EarRight_RotX,EarRight_RotY,EarRight_RotZ,EarRight_confidence\n";
        
        int frame_count = 0;
        auto original_time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();
        auto old_time = original_time;

        do
        {
            cout << "Restarting timer ...\n";
            old_time = chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count();

            k4a::capture sensor_capture;
            if (device.get_capture(&sensor_capture, chrono::milliseconds(K4A_WAIT_INFINITE)))
            {
                frame_count++;
                cout << "Start processing frame " << frame_count << endl;
                async(launch::async, [&]() { get_body_data(trk, sensor_capture, file, old_time, x_angle, y_angle, z_angle); });
                int wait = ceil(1000 / fps + old_time - chrono::duration_cast<chrono::milliseconds>(chrono::system_clock::now().time_since_epoch()).count());
                this_thread::sleep_for(std::chrono::milliseconds(wait));
            }
            else
            {
                // Handle timeout case
                cout << "Error! Get depth frame timeout!" << endl;
                break;
            }
        } while ((old_time - original_time) < (length_s * 1000));
        file.close();
        cout << "Finished body tracking processing!" << endl;

    }
    catch (const exception& e)
    {
        cerr << "Failed with exception:" << endl
            << "    " << e.what() << endl;
        return 1;
    }

    return 0;
}