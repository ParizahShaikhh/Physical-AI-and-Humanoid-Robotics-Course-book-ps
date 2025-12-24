# ADR-007: Documentation Technology Stack for Capstone Module

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-19
- **Feature:** 007-capstone-eval
- **Context:** Need to establish a consistent documentation technology stack for the capstone module that integrates seamlessly with existing course materials while supporting educational content requirements.

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security?
     2) Alternatives: Multiple viable options considered with tradeoffs?
     3) Scope: Cross-cutting concern (not an isolated detail)?
     If any are false, prefer capturing as a PHR note instead of an ADR. -->

## Decision

Use the existing Docusaurus 3.x documentation stack with Node.js 18+ and React-based components for the capstone module, maintaining consistency with previous modules.

- Framework: Docusaurus 3.x (React-based static site generator)
- Runtime: Node.js 18+ (for build and development)
- Content Format: Markdown with frontmatter (for educational content)
- Deployment: GitHub Pages (for accessibility and cost-effectiveness)
- Styling: Docusaurus default theme with course-specific customizations

## Consequences

### Positive

- Consistent user experience across all course modules
- Seamless integration with existing navigation and styling
- Familiar development workflow for course maintainers
- Excellent SEO and accessibility features built-in
- Strong documentation and community support
- Fast loading pages with optimized static generation
- Mobile-responsive design out of the box

### Negative

- Learning curve for contributors unfamiliar with React and MDX
- Potential performance issues with large documentation sets
- Dependency on Node.js ecosystem and version compatibility
- Limited customization without deeper React knowledge
- Build times may increase with more content

## Alternatives Considered

Alternative Stack A: Custom React application with Next.js + Tailwind CSS + Vercel
- More flexibility in UI/UX design
- Better performance optimization opportunities
- More modern tooling
- Why rejected: Would create inconsistency with existing modules and increase maintenance complexity

Alternative Stack B: Static Jekyll site with GitHub Pages
- Simpler technology stack
- Pure Markdown support
- Lightweight and fast
- Why rejected: Less interactive capabilities, limited educational content features, would require different navigation patterns than existing modules

Alternative Stack C: GitBook or similar documentation platform
- Managed hosting and maintenance
- Built-in search and collaboration features
- Easy content management
- Why rejected: Less customization control, potential vendor lock-in, doesn't match existing course architecture

## References

- Feature Spec: specs/007-capstone-eval/spec.md
- Implementation Plan: specs/007-capstone-eval/plan.md
- Related ADRs: ADR-004 (VLA Architecture)
- Evaluator Evidence: specs/007-capstone-eval/research.md