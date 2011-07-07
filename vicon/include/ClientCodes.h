//-----------------------------------------------------------------------------
//
//	Vicon RealTime SDK Sample Code: ClientCodes.h
//
//	This sample code is provided as an example of a a client-server 
//	application to obtain raw motion data from the Vicon RealTime Engine
//	for use in third-party data visualization, analysis, or manipulation 
//      software. 
//	This header file contains tokens and structures used by the
//	RealTime data stream. Also see ClientCodes.cpp, ExampleClient.cpp, and
//	ExampleClient.dsp.
//	
//	© 2000-2008 Vicon Motion Systems Limited. All rights reserved.
//-----------------------------------------------------------------------------

#ifndef __CLIENT_CODES__
#define __CLIENT_CODES__

#include <string>
#include <vector>
#include <functional>
#include <cassert>

class ClientCodes
{
 public:
  enum EType		
  {
    ERequest, 
    EReply
  };

  enum EPacket	
  {
    EClose, 
    EInfo, 
    EData, 
    EStreamOn, 
    EStreamOff
  };

  static const std::vector< std::string > MarkerTokens;
  static const std::vector< std::string > BodyTokens;

  static std::vector< std::string > MakeMarkerTokens()
  {
    std::vector< std::string > v;
    v.push_back("<P-X>");
    v.push_back("<P-Y>");
    v.push_back("<P-Z>");
    v.push_back("<P-O>");
    return v;
  }

  static std::vector< std::string > MakeBodyTokens()
  {
    std::vector< std::string > v;
    v.push_back("<A-X>");
    v.push_back("<A-Y>");
    v.push_back("<A-Z>");
    v.push_back("<T-X>");
    v.push_back("<T-Y>");
    v.push_back("<T-Z>");
    return v;
  }

  struct CompareNames : std::binary_function<std::string, std::string, bool>
    {
      bool operator()(const std::string & a_S1, const std::string & a_S2) const
      {
        std::string::const_iterator iS1 = a_S1.begin();
        std::string::const_iterator iS2 = a_S2.begin();

        while(iS1 != a_S1.end() && iS2 != a_S2.end())
          if(toupper(*(iS1++)) != toupper(*(iS2++))) return false;

        return a_S1.size() == a_S2.size();
      }
    };



};

class MarkerChannel
{
 public:

  int X;
  int Y;
  int Z;
  int O;
  std::string Name;

 MarkerChannel(std::string & a_rName) : X(-1), Y(-1), Z(-1), O(-1), Name(a_rName) {}

  int & operator[](int i)
  {
    switch(i)
      {
      case 0:		return X;
      case 1:		return Y;
      case 2:		return Z;
      case 3:		return O;
      default:	assert(false); return O;
      }
  }

  int operator[](int i) const
  {
    switch(i)
      {
      case 0:		return X;
      case 1:		return Y;
      case 2:		return Z;
      case 3:		return O;
      default:	assert(false); return -1;
      }
  }


  bool operator==(const std::string & a_rName) 
  {
    ClientCodes::CompareNames comparitor;
    return comparitor(Name, a_rName);
  }

};


class MarkerData
{
 public:
  double	X;
  double	Y;
  double	Z;
  bool	Visible;
};

class BodyChannel
{
 public:
	
  int RX;
  int RY;
  int RZ;
  int TX;
  int TY;
  int TZ;
  std::string Name;

 BodyChannel(std::string & a_rName) : RX(-1), RY(-1), RZ(-1), TX(-1), TY(-1), TZ(-1), Name(a_rName) {}

  int & operator[](int i)
  {
    switch(i)
      {
      case 0:		return RX;
      case 1:		return RY;
      case 2:		return RZ;
      case 3:		return TX;
      case 4:		return TY;
      case 5:		return TZ;
      default:	assert(false); return TZ;
      }
  }

  int operator[](int i) const
  {
    switch(i)
      {
      case 0:		return RX;
      case 1:		return RY;
      case 2:		return RZ;
      case 3:		return TX;
      case 4:		return TY;
      case 5:		return TZ;
      default:	assert(false); return -1;
      }
  }

  bool operator==(const std::string & a_rName) 
  {
    ClientCodes::CompareNames comparitor;
    return comparitor(Name, a_rName);
  }
};

class BodyData
{
 public:
  // Representation of body translation
  double	TX;
  double	TY;
  double	TZ;

  // Representation of body rotation
  // Quaternion
  double	QX;
  double	QY;
  double	QZ;
  double	QW;
  // Global rotation matrix
  double GlobalRotation[3][3];

  double EulerX;
  double EulerY;
  double EulerZ;
};
#endif
