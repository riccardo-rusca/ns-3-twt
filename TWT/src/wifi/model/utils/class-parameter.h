/*
 * Copyright (c) 2023 Politecnico di Torino
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Author: Francesco Raviglione <francescorav.es483@gmail.com>
 */

#ifndef CLASS_PARAMETER_H
#define CLASS_PARAMETER_H

#include <map>

namespace ns3
{
  template <class T>
  class classParameter {
    public:
      classParameter();
      classParameter(T initial_value) {m_valuemap[1]=initial_value;}
      classParameter(T initial_value, int num_elements) {
        for(int i=1;i<=num_elements;i++) {
          m_valuemap[i]=initial_value;
        }
      }

      void addValue(unsigned int classnumber,T value) {
        if(this->checkValueAvailable(classnumber)==false) {
          m_valuemap[classnumber]=value;
        } else {
          NS_FATAL_ERROR("Attempted to add an already existing value to a classParameter. Use changeValue() instead.");
        }
      }

      void changeValue(unsigned int classnumber,T value) {
        if(this->checkValueAvailable(classnumber)==true) {
          m_valuemap[classnumber]=value;
        } else {
          NS_FATAL_ERROR("Attempted to change a non-existing value in a classParameter. Use addValue() instead.");
        }
      }

      void removeValue(unsigned int classnumber) {
        if(this->checkValueAvailable(classnumber)==true) {
          m_valuemap.erase(classnumber);
        }
      }

      bool checkValueAvailable(unsigned int classnumber) {
        return m_valuemap.count(classnumber)>0;
      }

      T getValue(unsigned int classnumber) {
        return m_valuemap[classnumber];
      }

      void fillFromString(std::string values) {
        // This function expects a string in the form key1,value1#key2,value2#[...] to automatically fill the classParameter
        // Divide the string into tokens by using the "#" delimiter
        std::stringstream valuess(values);
        std::string token, key, value;
        while(getline(valuess,token,'#')) {
            std::stringstream keyvalue(token);
            getline(keyvalue,key,',');
            getline(keyvalue,value,',');

            // i = int
            // j = unsigned int
            // m = unsigned long int
            // d = double
            // f = float
            if(std::string(typeid(T).name())=="i") {
              m_valuemap[std::stoul(key)]=std::stoi(value);
            } else if(std::string(typeid(T).name())=="j" || std::string(typeid(T).name())=="m") {
              m_valuemap[std::stoul(key)]=std::stoul(value);
            } else if(std::string(typeid(T).name())=="d") {
              m_valuemap[std::stoul(key)]=std::stod(value);
            } else if(std::string(typeid(T).name())=="f") {
              m_valuemap[std::stoul(key)]=std::stof(value);
            } else {
              NS_FATAL_ERROR("Error. Called classParameter::fillFromString with unsupported type " << typeid(T).name() << "! You will need to fill in the classParameter yourself.");
            }
        }
      }

      void fillFromStringSequential(std::string values) {
        // This function expects a string in the form value1#value2#[...] to automatically fill the classParameter with increasing key values
        // Divide the string into tokens by using the "#" delimiter
        std::stringstream valuess(values);
        std::string token, value;
        unsigned int key=1;
        while(getline(valuess,token,'#')) {
            std::stringstream singlevalue(token);

            // i = int
            // j = unsigned int
            // m = unsigned long int
            // d = double
            // f = float
            if(std::string(typeid(T).name())=="i") {
              m_valuemap[key]=std::stoi(singlevalue.str());
            } else if(std::string(typeid(T).name())=="j" || std::string(typeid(T).name())=="m") {
              m_valuemap[key]=std::stoul(singlevalue.str());
            } else if(std::string(typeid(T).name())=="d") {
              m_valuemap[key]=std::stod(singlevalue.str());
            } else if(std::string(typeid(T).name())=="f") {
              m_valuemap[key]=std::stof(singlevalue.str());
            } else {
              NS_FATAL_ERROR("Error. Called classParameter::fillFromStringSequential with unsupported type " << typeid(T).name() << "! You will need to fill in the classParameter yourself.");
            }

            key++;
        }
      }

      size_t getSize(void) {return m_valuemap.size();}

      T getElementSum(void) {
        T accumulator{};

        for (auto const& x : m_valuemap)
        {
            accumulator+=x.second;
        }

        return accumulator;
      }
    private:
      std::map<unsigned int,T> m_valuemap;
  };
} // namespace ns3

#endif /* CLASS_PARAMETER_H */
