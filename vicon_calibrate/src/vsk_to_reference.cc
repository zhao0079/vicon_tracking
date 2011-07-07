#include <string>
#include <vector>
#include <fstream>

#include <XMLConfig.h>

using namespace std;

int main(int argc, char** argv)
{
  if (argc != 4)
    {
      puts("Usage: vsk_to_reference vsk_in positions_in reference_out");
      return -1;
    }

  string vsk(argv[1]);
  string positions(argv[2]);
  string reference(argv[3]);

  XMLConfig vsk_config;
  
  if (vsk_config.Load(vsk) != 0)
    {
      printf("Failed to load vsk file: %s\n", vsk.c_str());
      return -1;
    }
 
  if (!vsk_config.HasElement("/KinematicModel/MarkerSet/Markers/Marker"))
    {
      printf("Failed to locate Markers in vsk file %s\n",
             vsk.c_str());
      return -1;
    }

  int i = 0;
  string model_name;
  bool model_set = false;
  while (true)
    {
      XMLConfig* c = 
        vsk_config.GetChildrenAsRoot("/KinematicModel/MarkerSet/Markers", i);
  
      if (c == NULL)
        break;
      
      if (!model_set)
        {
          if (!c->HasAttribute("SEGMENT"))
            {
              printf("Failed to locate model name in vsk file %s",
                     vsk.c_str());
              return -1;
            }
          else
            {
              c->GetAttributeString("SEGMENT", model_name);
              model_set = true;
            }
        }
      
      delete c;
      i++;
    }
  
  i = 0;
  vector<string> names;
  
  puts("Finding markers");
  while (true)
    {
      XMLConfig* c = 
        vsk_config.GetChildrenAsRoot("/KinematicModel/MarkerSet/Markers", i);
      
      if (c == NULL)
        break;
      
      if (c->HasAttribute("STATUS"))
        {
          if ((!c->HasAttribute("NAME")) &&
              (!c->HasAttribute("POSITION")))
            {
              printf("Improperly formatted vsk file %s",
                     vsk.c_str());
              return -1;                 
            }
          
          string name;
          c->GetAttributeString(string("NAME"), name);
          names.push_back(model_name + string(":") + name);         
        }
      
      delete c;
      i++;
    }

  XMLConfig pos_config;
  
  if (pos_config.Load(positions) != 0)
    {
      printf("Failed to load positions file: %s\n", positions.c_str());
      return -1;
    }
 
  if (!pos_config.HasElement("/positions"))
    {
      printf("Failed to locate positions in positions file %s\n",
             positions.c_str());
      return -1;
    }
  
  i = 0;
  vector<float> values;
  
  puts("Finding positions");
  while (true)
    {
      XMLConfig* c = 
        pos_config.GetChildrenAsRoot("/positions", i);
      
      if (c == NULL)
        break;

      if (!c->HasElement("xyz"))
        {
          printf("Improperly formatted positions file %s",
                 positions.c_str());
          return -1;                 
        }
      
      float x = c->GetTupleFloat(string("xyz"), 0, 0);
      float y = c->GetTupleFloat(string("xyz"), 1, 0);
      float z = c->GetTupleFloat(string("xyz"), 2, 0);
      
      values.push_back(x);
      values.push_back(y);
      values.push_back(z);
     
      delete c;
      i++;
    }

  if (values.size() != 3*names.size())
    {
      printf("Inconsistent input: vsk provides %i markers, "
             "reference provides %i positions\n", (int)names.size(),
             (int)values.size()/3);
      return -1;
    }

  ofstream out(reference.c_str(), ios_base::out);

  out << "<reference>" << endl;
  out << "\t<markers>" << endl;
  
  for (unsigned int i = 0; i < names.size(); i++)
    {
      out << "\t\t<marker>" << endl;
      out << "\t\t\t<name>" << names[i] << "</name>" << endl;
      out << "\t\t\t<position>" 
          << values[3*i] << " "
          << values[3*i + 1] << " "
          << values[3*i + 2] << "</position>" << endl;
      out << "\t\t</marker>" << endl;
    }

  out << "\t</markers>" << endl;
  out << "</reference>" << endl;

  return 0;
}
