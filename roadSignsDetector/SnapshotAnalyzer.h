#include <opencv2/core/core.hpp>

#pragma once
class SnapshotAnalyzer
{
	public:
		SnapshotAnalyzer(const std::string);

		void SnapshotAnalyzer::Normalize();

		void SnapshotAnalyzer::CrossMatch();

		~SnapshotAnalyzer();

		static void ShowImage();

	private:
		void ConvertToSobel();
		
		void CannyThreshold();

		void CutMiddle();

};

