#ifndef _SCRIPT_CONTROLLER_H_
#define _SCRIPT_CONTROLLER_H_

#include <cstdint>

#ifdef __cplusplus
#include <vector>
#include <string>
#include <map>
#include "eeprom_controller.hpp"
#include "eeprom_cal_controller.hpp"
#include "eeprom_file_controller.hpp"

using typename std::size_t; 

class ScriptController 
{
public:
    void service(void);
    bool script_valid(void) { return m_scriptEeprom.validated();}
    const char* get_script_file(void) 
    {
        if(m_scriptEeprom.validated() && m_ScriptInFs)
        {
            return "babel_program.py";
        }
        return NULL;
    }
    void new_script(size_t len);
    void commit_script(void);
    bool verify_eeprom_valid(void); 
    bool verify_fs_script(void); 
    void init_fs(void); 
    float get_cal_data(char* key);
    bool validate_cal_data(void); 
    void populate_cal_data(void);
    void set_active_reader(bool script_active);
    bool confirm_eeprom(void); 
private:
    enum scriptAction {
        idle = 0,
        write = 1,
        validate = 2,
        read = 3 
    };
    scriptAction m_Action;
    static constexpr const char* m_ScriptFileName = "/_#!#_spiffs/babel_program.py";

    
    bool validate_script(eeprom_utils* eeprom, uint16_t new_crc); 

    bool m_ScriptInFs;
    eeprom_file_controller m_scriptEeprom = eeprom_file_controller(0, 1);
    eeprom_cal_controller m_calDataEeprom = eeprom_cal_controller(2000);

    constexpr static const char* m_Tag = "ScriptController";
};
#endif //#ifdef __cplusplus

#endif //_SCRIPT_CONTROLLER_H_
