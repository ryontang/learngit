
/*
 *
 *  Author: dks
 *  Funtion: Operate ini's configure file
 *
 */

#ifndef OPERATORINI_H
#define OPERATORINI_H
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <cstdlib>
#include <map>

using namespace std;

class ININode
{
public: ININode(string root, string key, string value)
    {
        this->root = root; this->key = key;
        this->value = value;
    }
    string root;
    string key;
    string value;
};

class SubNode
{
public:
    void InsertElement(string key, string value)
    {
        sub_node.insert(pair<string, string>(key, value));
    }
    map<string, string> sub_node;
};

class OperatorIni
{
public:
  OperatorIni();
  ~OperatorIni();
  int ReadINI(string path);
  string GetValue(string root, string key);
  vector<ININode>::size_type GetSize(){ return map_ini.size(); }
  vector<ININode>::size_type SetValue(string root, string key, string value);
  int WriteINI(string path);			//写入INI文件
  void Clear(){ map_ini.clear(); }	//清空
  void Travel();
private:
  map<string, SubNode> map_ini;		//INI文件内容的存储变量
};

#endif // OPERATORINI_H
