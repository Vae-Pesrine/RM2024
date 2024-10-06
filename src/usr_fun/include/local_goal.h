#include "ros/ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt32.h"
#include "usr_fun/refree.h"
#include <sys/ipc.h>
#include <sys/shm.h>
#include <string>
#include <string.h>


struct gimbal_send_msg{
    uint8_t header;
    float v_x;
    float v_y;
    float w_z;
    uint64_t config;
    float pose_x;
    float pose_y;
    float pose_z;
    float yaw;
    uint8_t tailer;
}__attribute__((packed));


usr_fun::refree refree_msg;



struct CamsenseConfig {
    uint64_t  receive_config;
    double    orientation_w;
    double    orientation_x;
    double    orientation_y;
    double    orientation_z;
    double    position_x;
    double    position_y;
    double    position_z;
    double    nav_at_aim_yaw;
    double    curr_aim_yaw;
    double    target_x;
    double    target_y;
    double    target_z;
    uint64_t  send_config;
};

enum CamsenseColor {
    CAMSENSE_COLOR_BLUE = 0x00000000,
    CAMSENSE_COLOR_RED = 0x00000002
};

key_t __gen_hash_key__(const std::string& __str__) {
    unsigned long __hash = 5381;
    const char* __cstr = __str__.c_str();
    int __c;
    while ((__c = *__cstr++))
        __hash = ((__hash << 5) + __hash) + __c;
    return (key_t)__hash;
}

void* __shm_alloc__(key_t __key__, size_t __size__) {
    int __shmid = shmget(__key__, __size__, IPC_CREAT | IPC_EXCL | 0666);
    if (__shmid == -1) {
        __shmid = shmget(__key__, 0, 0);
        if (__shmid == -1) { perror("shmget"); return NULL; }
    }
    void* __shm_ptr = shmat(__shmid, NULL, 0);
    if (__shm_ptr == (void*)-1) { perror("shmat"); return NULL; }

    return __shm_ptr;
}

void __shm_free__(key_t __key__) {
    int shmid = shmget(__key__, 0, 0);
    if (shmid == -1) { perror("shmget"); return; }
    if (shmctl(shmid, IPC_RMID, NULL) == -1) { perror("shmctl"); return; }
}

CamsenseConfig* SharedMemory(std::string __name__, size_t __num__ = 1UL) {
    key_t __shm_key = __gen_hash_key__(__name__);
    CamsenseConfig* __ptr = (CamsenseConfig*)__shm_alloc__(__shm_key, __num__ * sizeof(CamsenseConfig));
    memset(__ptr, 0, __num__ * sizeof(CamsenseConfig));
    return __ptr;
}

void SharedFree(std::string __name__) {
    key_t __shm_key = __gen_hash_key__(__name__);
    __shm_free__(__shm_key);
}