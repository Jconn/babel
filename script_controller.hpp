#ifndef _SCRIPT_CONTROLLER_H_
#define _SCRIPT_CONTROLLER_H_

#include <cstdint>

#ifdef __cplusplus
#include <vector>
#include <string>
#include "eeprom_controller.hpp"

using typename std::size_t; 

class ScriptController 
{
public:
    void service(void);
    bool script_valid(void) { return m_ScriptValid;}
    const char* get_script_file(void) 
    {
        if(m_FsInit && m_ScriptValid && m_ScriptCommited)
        {
            return "babel_program.py";
        }
        return NULL;
    }
    bool validate_script(uint16_t new_crc); 
    void write_script(const std::vector<uint8_t> &script); 
    void write_script(const uint8_t *payload, size_t len);
    void append_data(const std::vector <uint8_t> &data);
    void append_data(const uint8_t *payload, size_t len);
    void new_script(size_t len);
    void commit_script(void);
    bool verify_eeprom_valid(void); 
    bool verify_fs_script(void); 
    void init_fs(void); 
private:
    enum scriptAction {
        idle = 0,
        write = 1,
        validate = 2,
        read = 3 
    };

    scriptAction m_Action;
    static constexpr const char* m_ScriptFileName = "/_#!#_spiffs/babel_program.py";
    bool m_ScriptValid; 
    bool m_FsInit;
    bool m_ScriptCommited;
    uint16_t m_ScriptCrc;
    bool _validate_script(void);   
    size_t m_ScriptLength;
    eeprom_utils m_Eeprom;
    constexpr static const char* m_Tag = "ScriptController";
};
#endif //#ifdef __cplusplus

#endif //_SCRIPT_CONTROLLER_H_

