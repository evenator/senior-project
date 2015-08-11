#include <VP_Api/vp_api.h>
#include <VP_Api/vp_api_error.h>
#include <VP_Api/vp_api_stage.h>
#include <ardrone_tool/Video/video_com_stage.h>
#include <ardrone_tool/Com/config_com.h>

vp_api_io_pipeline_t pipeline;
vp_api_io_data_t out;
vp_api_io_stage_t stages[NB_STAGES];
video_com_config_t icc;
icc.com = COM_VIDEO();
icc.buffer_size = 100000;
icc.protocol = VP_COM_UDP;
COM_CONFIG_SOCKET_VIDEO(&icc.socket, VP_COM_CLIENT, VIDEO_PORT, wifi_ardrone_ip);
pipeline.nb_stages = 0;
stages[pipeline.nb_stages].type = VP_API_INPUT_SOCKET;
stages[pipeline.nb_stages].cfg = (void *)&icc;
stages[pipeline.nb_stages].funcs = video_com_funcs;
pipeline.nb_stages++;



res = vp_api_open(&pipeline, &pipeline_handle);
if( SUCCEED(res) )
{
int loop = SUCCESS;
out.status = VP_API_STATUS_PROCESSING;
while(loop == SUCCESS)
{
if( SUCCEED(vp_api_run(&pipeline, &out)) )
{
if( (out.status == VP_API_STATUS_PROCESSING || out.status ==
VP_API_STATUS_STILL_RUNNING) )
{
loop = SUCCESS;
}
}
else loop = -1; // Finish this thread
}
vp_api_close(&pipeline, &pipeline_handle);
}

