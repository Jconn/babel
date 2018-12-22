#ifndef _SCRIPT_CONTROLLER_H_
#define _SCRIPT_CONTROLLER_H_

class ScriptController 
{
public:
    void service(void);
    bool script_valid(void) { return m_ScriptValid;}
    char* get_script_file(void) {return m_ScriptFileName.to_cstr();}
    bool validate_script(uint16_t new_crc); 
    void write_script(const std::vector<uint8_t> &script); 
    void write_script(const uint8_t *payload, size_t len);
    void append_data(const std::vector <uint8_t> &data);
    void append_data(const uint8_t *payload, size_t len);
    void new_script(size_t len) {m_ScriptLength = len;}

private:
    enum scriptAction {
        idle = 0,
        write = 1,
        validate = 2,
        read = 3 
    };

    scriptAction m_Action;
    std::string m_ScriptFileName;
    bool m_ScriptValid; 
    uint16_t m_ScriptCrc;
    bool _validate_script(void);   
    size_t m_ScriptLength;
    eeprom_utils m_Eeprom;

};

#endif //_SCRIPT_CONTROLLER_H_

