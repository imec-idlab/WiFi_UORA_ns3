#ifndef PROGRESS_BAR_H
#define PROGRESS_BAR_H

#include <ns3/core-module.h>

namespace ns3 {

class ProgressBar {
public:
  ProgressBar(Time totalTime);
  ~ProgressBar();
  void PrintProgress();
private:
  Time m_totalTime;
  Time m_step;
};

}

#endif /* PROGRESS_BAR_H */