/*****************************************************************************
* Copyright (c) 2016 BFFT Gesellschaft fuer Fahrzeugtechnik mbH.             *
* All rights reserved.                                                       *
******************************************************************************
* Author:: spiesra   Date:: 2016-04-14                                       *
* $LastChangedBy::          $  $Date::                     $ $Rev::        $ *
*****************************************************************************/

#include <sstream>
#include <map>

#undef GetObject

template <class T> class PropertyHelper
{
public:
    PropertyHelper(T* pFilter)
    {
        m_parentFilter = pFilter;
    };

    ~PropertyHelper() {};

    /* Adds Property to the Filter
    name: name of the Property
    defaultValue: default Value of the Property
    pstring: Pointer to the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No description */
    void addProperty(cString name, cString defaultValue, cString* pstring, cString description = "No description", tBool isChangeable = tFalse)
    {
        *pstring = defaultValue;
        m_parentFilter->SetPropertyStr(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_stringmap[name] = pstring;
    }
    /* Adds Property to the Filter
    name: name of the Property
    defaultValue: default Value of the Property
    pinteger: Pointer to the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No description */
    void addProperty(cString name, int defaultValue, int* pinteger, cString description = "No description", tBool isChangeable = tFalse, cString valuestring = "No Value List")
    {
        *pinteger = defaultValue;
        m_parentFilter->SetPropertyInt(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        if (valuestring != "No Value List")
        {
            m_parentFilter->SetPropertyStr(name + NSSUBPROP_VALUELIST, valuestring);
        }
        m_intmap[name] = pinteger;
    }
    /* Adds Property to the Filter
    name: name of the Property
    defaultValue: default Value of the Property
    pfloat: Pointer to the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No description */
    void addProperty(cString name, tFloat64 defaultValue, tFloat64* pfloat, cString description = "No description", tBool isChangeable = tFalse)
    {
        *pfloat = defaultValue;
        m_parentFilter->SetPropertyFloat(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_floatmap[name] = pfloat;
    }
    /* Adds Property to the Filter
    name: name of the Property
    defaultValue: default Value of the Property
    pbool: Pointer to the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No description */
    void addProperty(cString name, bool defaultValue, bool* pbool, cString description = "No description", tBool isChangeable = tFalse)
    {
        *pbool = defaultValue;
        m_parentFilter->SetPropertyBool(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_boolmap[name] = pbool;
    }
    /* Adds Filename Property to Filter
    name: name of the Property
    defaultValue: default Value of the Property as cString
    pfilename: Pointer to the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No description
    extension: extension of the file ("Name of Extension (*.extension)", e.g.: "Dat Files (*.dat)")
    updateProperty and updateallProperty will save an absolute Path into the assigned Member Variable as cFilename*/
    void addProperty(cString name, cString defaultValue, cFilename* pfilename, cString description = "No description", tBool isChangeable = tFalse, cString extension = "No Extension")
    {
        *pfilename = defaultValue;
        m_parentFilter->SetPropertyStr(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_FILENAME, tTrue);
        if (extension != "No Extension")
        {
            m_parentFilter->SetPropertyStr(name + NSSUBPROP_FILENAME NSSUBSUBPROP_EXTENSIONFILTER, extension);
        }
        m_filenamemap[name] = pfilename;
    }
    /* Adds TimeStamp Property to Filter
    name: name of Property
    defaultValue: default Value of the Property as int
    ptimestamp: Pointer to the the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No Description
    updateProperty and updateallProperty will save a tTimestamp in microseconds in the assigned Member Variable */
    void addProperty(cString name, int defaultValue, tTimeStamp* ptimestamp, cString description = "No description", tBool isChangeable = tFalse)
    {
        *ptimestamp = defaultValue;
        description += " [ms]";
        m_parentFilter->SetPropertyInt(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_timestampintmap[name] = ptimestamp;
    }
    /* Adds TimeStamp Property to Filter
    name: name of Property
    defaultValue: default Value of the Property as int
    ptimestamp: Pointer to the the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No Description
    updateProperty and updateallProperty will save a tTimestamp in microseconds in the assigned Member Variable */
    void addProperty(cString name, float defaultValue, tTimeStamp* ptimestamp, cString description = "No description", tBool isChangeable = tFalse)
    {
        *ptimestamp = defaultValue;
        description += " [s]";
        m_parentFilter->SetPropertyFloat(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_timestampfloatmap[name] = ptimestamp;
    }
    /* Adds TimeStamp Property to Filter
    name: name of Property
    defaultValue: default Value of the Property as cString
    pcolor: Pointer to the the Member variable to save the Value of the Property in
    description: description of the Property, default Value: No Description
    updateProperty and updateallProperty will save a cColor in the assigned Member Variable */
    void addProperty(cString name, cString defaultValue, cColor* pcolor, cString description = "No description", tBool isChangeable = tFalse)
    {
        String2Color(defaultValue.GetPtr(), *pcolor);
        m_parentFilter->SetPropertyStr(name, defaultValue);
        m_parentFilter->SetPropertyStr(name + NSSUBPROP_DESCRIPTION, description);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_COLOR, tTrue);
        m_parentFilter->SetPropertyBool(name + NSSUBPROP_ISCHANGEABLE, isChangeable);
        m_colormap[name] = pcolor;
    }
    /* updates Property Value in the assigned Membervariable
    name: name of the Property */
    void updateProperty(cString name)
    {
        if (m_stringmap.count(name) == 1)
        {
            std::map<cString, cString *>::iterator it = m_stringmap.find(name);
            *(it->second) = m_parentFilter->GetPropertyStr(name);
        }
        if (m_intmap.count(name) == 1)
        {
            std::map<cString, int *>::iterator it = m_intmap.find(name);
            *(it->second) = m_parentFilter->GetPropertyInt(name);
        }
        if (m_floatmap.count(name) == 1)
        {
            std::map<cString, tFloat64 *>::iterator it = m_floatmap.find(name);
            *(it->second) = m_parentFilter->GetPropertyFloat(name);
        }
        if (m_boolmap.count(name) == 1)
        {
            std::map<cString, bool *>::iterator it = m_boolmap.find(name);
            *(it->second) = m_parentFilter->GetPropertyBool(name);
        }
        if (m_filenamemap.count(name) == 1)
        {
            cFilename PropertyString;
            std::map<cString, cFilename *>::iterator it = m_filenamemap.find(name);
            PropertyString.Set(m_parentFilter->GetPropertyStr(name));
            ADTF_GET_CONFIG_FILENAME(PropertyString);
            *(it->second) = PropertyString;
        }
        if (m_timestampintmap.count(name) == 1)
        {
            std::map<cString, tTimeStamp *>::iterator it = m_timestampintmap.find(name);
            *(it->second) = m_parentFilter->GetPropertyInt(name) * 1000;
        }
        if (m_timestampfloatmap.count(name) == 1)
        {
            std::map<cString, tTimeStamp *>::iterator it = m_timestampfloatmap.find(name);
            *(it->second) = m_parentFilter->GetPropertyFloat(name) * 1000000;
        }
        if (m_colormap.count(name) == 1)
        {
            std::map<cString, cColor *>::iterator it = m_colormap.find(name);
            String2Color(m_parentFilter->GetPropertyStr(name), *(it->second));
        }
    }
    /* Updates all Properties to their assigned Membervariable */
    void updateallProperties()
    {
        for (std::map<cString, cString *>::iterator it = m_stringmap.begin(); it != m_stringmap.end(); ++it)
        {
            *(it->second) = m_parentFilter->GetPropertyStr(it->first);
        }
        for (std::map<cString, int *>::iterator it = m_intmap.begin(); it != m_intmap.end(); ++it)
        {
            *(it->second) = m_parentFilter->GetPropertyInt(it->first);
        }
        for (std::map<cString, tFloat64 *>::iterator it = m_floatmap.begin(); it != m_floatmap.end(); ++it)
        {
            *(it->second) = m_parentFilter->GetPropertyFloat(it->first);
        }
        for (std::map<cString, bool *>::iterator it = m_boolmap.begin(); it != m_boolmap.end(); ++it)
        {
            *(it->second) = m_parentFilter->GetPropertyBool(it->first);
        }
        for (std::map<cString, cFilename *>::iterator it = m_filenamemap.begin(); it != m_filenamemap.end(); ++it)
        {
            cFilename PropertyString;
            PropertyString.Set(m_parentFilter->GetPropertyStr(it->first));
            ADTF_GET_CONFIG_FILENAME(PropertyString);
            *(it->second) = PropertyString;
        }
        for (std::map<cString, tTimeStamp *>::iterator it = m_timestampintmap.begin(); it != m_timestampintmap.end(); ++it)
        {
            *(it->second) = m_parentFilter->GetPropertyInt(it->first) * 1000;
        }
        for (std::map<cString, tTimeStamp *>::iterator it = m_timestampfloatmap.begin(); it != m_timestampfloatmap.end(); ++it)
        {
            *(it->second) = m_parentFilter->GetPropertyFloat(it->first) * 1000000;
        }
        for (std::map<cString, cColor *>::iterator it = m_colormap.begin(); it != m_colormap.end(); ++it)
        {
            String2Color(m_parentFilter->GetPropertyStr(it->first), *(it->second));
        }
    }

    tResult String2Color(const tChar* strColor, cColor& oColor)
    {
        if (strColor[0] != '#')
        {
            RETURN_ERROR(ERR_INVALID_ARG);
        }

        tInt r, g, b, a;
        r = g = b = 0;
        a = 255;

        for (tUInt nIdx = 1; nIdx < strlen(strColor); nIdx++)
        {
            tInt nVal = 0;
            tChar cChar = strColor[nIdx];

            switch (cChar)
            {
            case '0': case '1': case '2': case '3': case '4':
            case '5': case '6': case '7': case '8': case '9':
                nVal = cChar - '0';
                break;
            case 'a': case 'b': case 'c': case 'd': case 'e': case 'f':
                nVal = cChar - 'a' + 10;
                break;
            case 'A': case 'B': case 'C': case 'D': case 'E': case 'F':
                nVal = cChar - 'A' + 10;
                break;
            default:
                RETURN_ERROR(ERR_INVALID_ARG);
            }

            switch (nIdx)
            {
            case 1: r = nVal * 16; break;
            case 2: r = r + nVal; break;
            case 3: g = nVal * 16; break;
            case 4: g = g + nVal; break;
            case 5: b = nVal * 16; break;
            case 6: b = b + nVal; break;
            case 7: a = nVal * 16; break;
            case 8: a = a + nVal; break;
            }
        }
        oColor.Set(r, g, b, a);
        RETURN_NOERROR;
    }

private:
    T* m_parentFilter;
    std::map<cString, cString *> m_stringmap;
    std::map<cString, int *> m_intmap;
    std::map<cString, tFloat64 *> m_floatmap;
    std::map<cString, bool *> m_boolmap;
    std::map<cString, cFilename *> m_filenamemap;
    std::map<cString, tTimeStamp *> m_timestampfloatmap;
    std::map<cString, tTimeStamp *> m_timestampintmap;
    std::map<cString, cColor *> m_colormap;
};

