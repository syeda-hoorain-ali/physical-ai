import { Navbar } from "@site/src/components/navbar";
import { ScrollVelocity } from "@site/src/components/ui/scroll-based-velocity";
import { ChapterTimeline } from "@site/src/components/home-page-sections/chapter-timeline";
import { HeroSection } from "@site/src/components/home-page-sections/hero-section";
import { FeaturesSection } from "@site/src/components/home-page-sections/features-section";
import { AnimatedListSection } from "@site/src/components/home-page-sections/animated-list-section";
import { FooterCTASection } from "@site/src/components/home-page-sections/cta-section";


const HomePage = () => {
  return (
    <div className="dark min-h-screen bg-background">
      <Navbar />
      <HeroSection />

      {/* Scroll Velocity Section */}
      <section className="relative border-y border-border/30 bg-muted/30 py-4">
        <ScrollVelocity text="Physical AI • Humanoid Robots • Embodied Intelligence" velocity={25} />
      </section>

      <FeaturesSection />
      <AnimatedListSection />
      <ChapterTimeline />
      <FooterCTASection />
    </div>
  );
};

export default HomePage;
