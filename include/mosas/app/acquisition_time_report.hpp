#ifndef MOSAS_APP_ACQUISITION_TIME_REPORT_HPP
#define MOSAS_APP_ACQUISITION_TIME_REPORT_HPP
#include <mosas/domain/sequence_time.hpp>
#include <iosfwd>

namespace mosas::app {
/** @brief Write timing quality without implying hardware synchronization or correcting acquisition.
 * @param output Synchronous exit log stream; caller handles stream failures.
 * @param source Human-readable source name.
 * @param timing Stable diagnostic snapshot after stopping the source.
 */
void write_acquisition_time_report(std::ostream& output, const char* source,
                                   const domain::SequenceTimeDiagnostics& timing);
}
#endif
