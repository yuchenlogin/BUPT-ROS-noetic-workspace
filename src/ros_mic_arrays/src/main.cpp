#include "alsa_recorder.h"
#include <ros/ros.h>
#include "hlw.h"
#include "std_msgs/String.h"
#include "std_msgs/UInt8MultiArray.h"
#include <signal.h>
#include <vector>
#include <sstream>
#include <ros/package.h>
#include <fstream>
#include <jsoncpp/json/json.h> 
#include "ros_mic_arrays/setBeamSrv.h"

void test_ivw_fn(short angle, short channel, float power, short CMScore, short beam, char *param1, void *param2, void *userData)
{	
    std::vector<ros::Publisher> pubs = *(std::vector<ros::Publisher> *)userData;
    std_msgs::String msg;
    std::ostringstream ss;
    ss << "{'key_word': 'lu3', 'score': '" << int(CMScore) << "', 'angle': '" << int(angle) << "'}";
    ROS_INFO_STREAM(ss.str());
    msg.data = ss.str();
    pubs[0].publish(msg);
}

void test_recog_audio_fn(const void *audioData, unsigned int audioLen, int param1, const void *param2, void *userData)
{
    std::vector<ros::Publisher> pubs = *(std::vector<ros::Publisher> *)userData;
    if (pubs[0].getNumSubscribers() > 0) {
        std_msgs::UInt8MultiArray msg;
        for (size_t i = 0; i < audioLen; i++) {
                msg.data.push_back(*((char *)audioData + i));
        }
        pubs[1].publish(msg);
    }
}

void shutdown(int sig){
    ros::shutdown();    
}

bool set_real_beam(CAE_HANDLE cae_handle, 
                    ros_mic_arrays::setBeamSrv::Request &req, 
                    ros_mic_arrays::setBeamSrv::Response &res) {
    CAESetRealBeam(cae_handle, req.beamID);
    ROS_INFO("Manual set beam of %d\n", req.beamID);
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "alsa_node", ros::init_options::NoSigintHandler);

    ros::NodeHandle node;
    const ros::Publisher &wake_up_pub = node.advertise<std_msgs::String>("/micarrays/wakeup", 1000);
    const ros::Publisher &audio_stream_pub = node.advertise<std_msgs::UInt8MultiArray>("/audio/stream", 1000);

    std::vector<ros::Publisher> pubs{wake_up_pub, audio_stream_pub};

    auto package_path = ros::package::getPath("ros_mic_arrays");

    std::ostringstream conf_path_ss;
    conf_path_ss << package_path << "/bin/hlw.ini";

    std::ostringstream param_path_ss;
    param_path_ss << package_path << "/bin/hlw.param";

    AlsaRecorder rec("default", 8, SND_PCM_FORMAT_S32_LE, 16000, 1024, nullptr);

    auto conf_path_str = conf_path_ss.str();
    auto param_path_str = param_path_ss.str();

    auto conf_path = conf_path_str.c_str();
    auto param_path = param_path_str.c_str();

    CAE_HANDLE cae_handle;

    //0 调试  1信息  2错误
    // CAESetShowLog(1);

    int ret;
    ret = CAENew(&cae_handle, conf_path, test_ivw_fn, nullptr, test_recog_audio_fn, param_path, &pubs);
    if (ret != 0) {
        ROS_INFO_STREAM("CAENew error rv != 0, rv == " << ret);
		return -1;
	}

    boost::function<bool(ros_mic_arrays::setBeamSrv::Request&, ros_mic_arrays::setBeamSrv::Response&)> set_real_beam_func = boost::bind(&set_real_beam, cae_handle, _1, _2);
    ros::ServiceServer set_real_beam_service = node.advertiseService("/ros_mic_arrays/set_real_beam", set_real_beam_func);

    Json::Value root;
    Json::Reader reader;
    std::ifstream ifs("/home/lemon/.lejuconfig/iflyos_config.json");

    reader.parse(ifs, root);
    
    auto sn_str = root["hlw_sn"].asString();
    char *sn = const_cast<char*>(sn_str.c_str()); 

    ret = CAEAuth(sn);
    if ((ret != 0) && (ret != 70508)) {
        ROS_ERROR_STREAM("hlw_sn auth error ret: " << ret);
        return -1;
	} else {
        ROS_INFO_STREAM("hlw_sn auth ok ... (" << ret << ")");
	}
    
    auto fun = std::bind(CAEAudioWrite, cae_handle, std::placeholders::_1, std::placeholders::_2);
    signal(SIGINT, shutdown);
    rec.stratRecording(fun);
    return 0;
}
