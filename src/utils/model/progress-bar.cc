#include <unistd.h>

#ifdef __linux__
#include <sys/prctl.h>
#endif

#include "progress-bar.h"

namespace ns3 {
  ProgressBar::ProgressBar(Time totalTime) : m_totalTime(totalTime), m_step(Seconds (1))
    {
      PrintProgress();
    }

  void ProgressBar::PrintProgress ()
    {
      uint32_t percent = 100 * Simulator::Now ().GetSeconds () / m_totalTime.GetSeconds ();
      std::ostringstream s;
      s << percent << "%, "
        << (uint32_t)Simulator::Now ().GetSeconds () << "/"
        << (uint32_t)m_totalTime.GetSeconds ();

      if (isatty (2))
        {
          std::cerr << '\r' << s.str() << " seconds passed.";
        }
#ifdef __linux__
      prctl (PR_SET_NAME, s.str ().c_str ());
#endif

      if (Simulator::Now () + m_step < m_totalTime)
        {
          Simulator::Schedule (m_step, &ProgressBar::PrintProgress, this);
        }
    }

  ProgressBar::~ProgressBar()
    {
      if (isatty (2))
        {
          std::cerr << "\r\033[K";
        }
    }
}
