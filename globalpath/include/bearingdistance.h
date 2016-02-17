#ifndef global_map
#define global_map

namespace bearingdistane{
	class bearingdistance{
		private:
			const float pi = 3.14159265;
			const float earthradius = 6371000.000;
			const float RAD_TO_DEG = 180.0/pi;
			const float DEG_TO_RAD = pi/180.0;

		public:
			float checkradius;
			float pathlength;
			
			bearingdistance();
			~bearingdistance();

			float distancebetweengpspoints();
			float bearingbetweengpspoints();

			void loadtpf();
			void setcheckradius();

			float distancefromgpspoint();
			float bearingtogpspoint();
		
			float distancefromallgpspoints();
			float getpathlength();
			float remainingdistance();
		
			int currentgpspointindex();
			int tpfsize();
	};
}

#endif